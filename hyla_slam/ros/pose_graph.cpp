#include "pose_graph.hpp"

namespace hyla_slam {

PoseGraph::PoseGraph(const rclcpp::NodeOptions &opts)
: rclcpp::Node("hyla_slam", opts)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); // prevent warnings from pcl::conversions being printed constantly

    // handle parameters
    param_listener_ = std::make_shared<hyla_slam::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // create ROS stuff
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.point_cloud_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&PoseGraph::receiveScan, this, std::placeholders::_1)
    );

    auto localization_params = params_.localization;
    localization_config_.voxel_size = localization_params.voxel_size;
    localization_config_.max_range = localization_params.max_range;
    localization_config_.min_range = localization_params.min_range;
    localization_config_.max_points_per_voxel = localization_params.max_points_per_voxel;
    localization_config_.min_motion_th = localization_params.min_motion_th;
    localization_config_.initial_threshold = localization_params.initial_threshold;
    localization_config_.max_num_iterations = localization_params.max_num_iterations;
    localization_config_.convergence_criterion = localization_params.convergence_criterion;
    localization_config_.max_num_threads = localization_params.max_num_threads;
    localization_config_.deskew = localization_params.deskew;
    localizer_ = std::make_unique<hyla_kiss::HylaKiss>(localization_config_);

    sam_ = std::make_unique<hyla_sam::HylaSam>();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "poses",
        10
    );

    RCLCPP_INFO(this->get_logger(), "Up and ready!");
}


void PoseGraph::receiveScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_msg)
{
    if (nodes_.empty()) {
        nodes_.push_back(SamNode(Sophus::SE3d(), Sophus::SE3d(), raw_msg));
    }

    auto start {std::chrono::high_resolution_clock::now()};
    
    // check to make sure data is expressed in the proper frame
    if (raw_msg->header.frame_id != params_.localization_frame) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Incoming sensor data is in frame " << raw_msg->header.frame_id << " but localization_frame is " << params_.localization_frame << "; these should match. No localization update will be performed.");
        return;
    }

    // register the frame to update odom
    localizer_->registerFrame(kiss_icp_ros::utils::PointCloud2ToEigen(raw_msg));

    // get the pose out
    Sophus::SE3d pose_estimate {localizer_->pose()};

    // accumulate odometry (both in general and since last update)
    if (last_pose_.has_value()) {
        dist_since_update_ += (pose_estimate.translation() - last_pose_.value().translation()).norm();

        RCLCPP_DEBUG_STREAM(this->get_logger(), "d: " << dist_since_update_ << "m");

        if (dist_since_update_ >= 1.0) {
            
            auto rel_odom {nodes_.back().odom_pose.inverse() * pose_estimate};
            nodes_.push_back(SamNode(pose_estimate, rel_odom, raw_msg));
            dist_since_update_ = 0.0;

            sam_->addOdomFactor(rel_odom);

            // add loop closures
            int recent_excluded {5};
            double closeness {2.0};

            std::size_t effective_size = (nodes_.size() > static_cast<size_t>(recent_excluded)) ? nodes_.size() - static_cast<size_t>(recent_excluded) : 0;

            for (std::size_t i = 0; i < effective_size; ++i) {
                // check closeness to previous poses
                if ((nodes_.at(i).odom_pose.translation() - pose_estimate.translation()).norm() < closeness) {
                    // TODO add loop closure
                    

                    RCLCPP_INFO_STREAM(this->get_logger(), "Would add loop closure between " << static_cast<int>(i) << " and new pose!");
                }
            }

            // performing the optimization every so often
            if (nodes_.size() % 5 == 0) {
                auto opt_start {std::chrono::high_resolution_clock::now()};
                RCLCPP_INFO(this->get_logger(), "Trying to optimize graph!");
                auto values {sam_->optimize()};
                auto opt_end {std::chrono::high_resolution_clock::now()};
                std::chrono::duration<double, std::milli> opt_elapsed {opt_end - opt_start};
                RCLCPP_INFO_STREAM(this->get_logger(), "Finished opt in " << opt_elapsed.count() << "ms");

                // create markers for poses
                auto graph {sam_->getGraph()};
                publishSamMarkers(values, graph);
            }
        }
    }


    // get out map representation and publish it











    // broadcast the raw tf
    geometry_msgs::msg::TransformStamped robot_tf;
    robot_tf.transform = tf2::sophusToTransform(pose_estimate);
    robot_tf.header.frame_id = params_.fixed_frame;
    robot_tf.child_frame_id = raw_msg->header.frame_id;
    robot_tf.header.stamp = raw_msg->header.stamp;
    tf_broadcaster_->sendTransform(robot_tf);

    // update stuff for next iteration
    last_pose_ = pose_estimate;

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "receiveScan: " << elapsed.count() << "ms");
}

void PoseGraph::publishSamMarkers(const gtsam::Values &values, const gtsam::NonlinearFactorGraph &graph)
{
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = params_.fixed_frame;  // Use your desired frame
    edge_marker.header.stamp = this->now();
    edge_marker.ns = "graph_edges";
    edge_marker.id = 0;  // Single marker ID for all edges
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.scale.x = 0.02;  // Line thickness
    edge_marker.color.r = 0.0;
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 1.0;
    edge_marker.color.a = 1.0;

    // Iterate through all factors in the graph
    for (const auto& factor : graph) {
        auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
        if (between_factor) {
            // Extract the keys (poses) connected by the factor
            gtsam::Key key1 = between_factor->key1();
            gtsam::Key key2 = between_factor->key2();

            // Retrieve the poses from the values
            if (values.exists<gtsam::Pose3>(key1) && values.exists<gtsam::Pose3>(key2)) {
                gtsam::Pose3 pose1 = values.at<gtsam::Pose3>(key1);
                gtsam::Pose3 pose2 = values.at<gtsam::Pose3>(key2);

                // Create points for the line
                geometry_msgs::msg::Point p1, p2;
                p1.x = pose1.translation().x();
                p1.y = pose1.translation().y();
                p1.z = pose1.translation().z();
                p2.x = pose2.translation().x();
                p2.y = pose2.translation().y();
                p2.z = pose2.translation().z();

                // Add the points to the edge marker
                edge_marker.points.push_back(p1);
                edge_marker.points.push_back(p2);
            }
        }
    }

    markers.markers.push_back(edge_marker);

    for (const auto &key_value : values) {
        gtsam::Key key = key_value.key;
        if (!values.exists<gtsam::Pose3>(key)) {
            continue;
        }

        gtsam::Pose3 pose {values.at<gtsam::Pose3>(key)};
        Eigen::Vector3d trans {pose.translation()};
        Eigen::Quaterniond rot {pose.rotation().toQuaternion()};

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = params_.fixed_frame;
        marker.header.stamp = this->now();
        marker.id = key;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = trans.x();
        marker.pose.position.y = trans.y();
        marker.pose.position.z = trans.z();
        marker.pose.orientation.x = rot.x();
        marker.pose.orientation.y = rot.y();
        marker.pose.orientation.z = rot.z();
        marker.pose.orientation.w = rot.w();
        marker.scale.x = 0.2;  // Arrow length
        marker.scale.y = 0.02; // Arrow shaft diameter
        marker.scale.z = 0.02; // Arrow head diameter
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        markers.markers.push_back(marker);
    }

    marker_pub_->publish(markers);
}

} // namespace hyla_slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node (std::make_shared<hyla_slam::PoseGraph>(rclcpp::NodeOptions()));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
