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
}

void PoseGraph::receiveScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_msg)
{
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

    // TODO need to accumulate odometry (both in general and since last update)

    // TODO need to perform scan registration to get out the factors for registration

    // broadcast the raw tf
    geometry_msgs::msg::TransformStamped robot_tf;
    robot_tf.transform = tf2::sophusToTransform(pose_estimate);
    robot_tf.header.frame_id = params_.fixed_frame;
    robot_tf.child_frame_id = raw_msg->header.frame_id;
    robot_tf.header.stamp = raw_msg->header.stamp;
    tf_broadcaster_->sendTransform(robot_tf);

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_INFO_STREAM(this->get_logger(), "receiveScan: " << elapsed.count() << "ms");
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
