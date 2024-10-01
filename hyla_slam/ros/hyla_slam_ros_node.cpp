#include "hyla_slam_ros_node.hpp"

namespace hylacomylus {

RosNode::RosNode(const rclcpp::NodeOptions &opts)
: rclcpp::Node("hyla_slam", opts), counter_(0), slam_enabled_(false)
{
    RCLCPP_INFO(this->get_logger(), "HERE");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); // prevent warnings from pcl::conversions being printed constantly

    // handle parameters
    param_listener_ = std::make_shared<hyla_slam::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // create the configs and the two pipelines
    auto mapping_params = params_.mapping;
    mapping_config_.fixed_frame = params_.fixed_frame;
    mapping_config_.robot_frame = params_.robot_frame;
    mapping_config_.odom_frame = params_.odom_frame;
    mapping_config_.chunk_discretization = mapping_params.chunk_discretization;
    mapping_config_.chunk_load_dir = mapping_params.data_dir;
    mapping_config_.half_side_length = mapping_params.half_side_length;
    mapping_config_.active_mapping = params_.active_mapping;
    mapping_config_.persist_recent_chunks = params_.persist_recent_chunks;
    mapper_ = std::make_unique<Hylacomylus>(mapping_config_);

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
    localizer_ = std::make_unique<HylaKiss>(localization_config_);

    // create ROS stuff
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::optional<rclcpp::QoS> sensor_data_qos {std::nullopt};
    if (params_.sensor_drivers_bridged) {
        sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.point_cloud_topic,
        sensor_data_qos.value_or(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()),
        std::bind(&RosNode::handleNewFrame, this, std::placeholders::_1)
    );

    reference_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/reference_cloud",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
    );

    frame_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robot_map",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
    );

    enable_slam_server_ = this->create_service<std_srvs::srv::Trigger>(
        "enable_slam",
        std::bind(&RosNode::enableSLAM, this, std::placeholders::_1, std::placeholders::_2)
    );

    disable_slam_server_ = this->create_service<std_srvs::srv::Trigger>(
        "disable_slam",
        std::bind(&RosNode::disableSLAM, this, std::placeholders::_1, std::placeholders::_2)
    );

    unload_data_server_ = this->create_service<std_srvs::srv::Trigger>(
        "unload_data",
        std::bind(&RosNode::unloadData, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Up and ready!");
}

void RosNode::enableSLAM(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (slam_enabled_ == true) {
        RCLCPP_WARN_STREAM(this->get_logger(), "SLAM already enabled!");
    }
    slam_enabled_ = true;
    response->success = true;
}

void RosNode::disableSLAM(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (slam_enabled_ == false) {
        RCLCPP_WARN_STREAM(this->get_logger(), "SLAM already disabled!");
    }
    slam_enabled_ = false;
    response->success = true;
}

void RosNode::unloadData(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    mapper_->dumpMemoryData();
    RCLCPP_INFO(this->get_logger(), "Unloaded data in memory to disk!");
    last_localization_update_point_ = nullptr;
    last_mapping_update_point_ = nullptr;
    response->success = true;
}

double RosNode::getDisplacement(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    return (p1 - p2).norm();
}

void RosNode::handleNewFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_msg)
{
    if (!slam_enabled_) { return; }

    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "[THROTTLED] Inside handle frame : " << counter_ << ".");

    // TODO transform cloud into "localization frame" (like base_link) if it is not already
    auto transformed_msg {RosNode::transformPointCloud(*raw_msg, params_.robot_frame)};

    if (!transformed_msg.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform cloud into robot frame! No SLAM update performed.");
        return;
    }

    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg(std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_msg.value()));  

    PointCloud::Ptr raw_point_cloud (new PointCloud);
    pcl::fromROSMsg(*msg, *raw_point_cloud);

    // update cloud for localization
    auto pose_prior = localizer_->pose();
    if (last_localization_update_point_ == nullptr || 
        (last_localization_update_point_ != nullptr &&
        getDisplacement(pose_prior.translation(), *last_localization_update_point_) > params_.localization_threshold)
    ) {
        auto map {mapper_->map()};
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting map in localizer (map has " << map->points.size() << " points).");
        // TODO
        if (map->points.size() > 0) {
            auto map_msg {std::make_shared<sensor_msgs::msg::PointCloud2>()};
            pcl::toROSMsg(*map, *map_msg);
            localizer_->setMap(kiss_icp_ros::utils::PointCloud2ToEigen(map_msg));

            last_localization_update_point_ = std::make_unique<Eigen::Vector3d>(pose_prior.translation());

            // map_msg->header.frame_id = params_.fixed_frame;
            // reference_publisher_->publish(*map_msg);
        }
    }

    // register the frame
    localizer_->registerFrame(kiss_icp_ros::utils::PointCloud2ToEigen(msg));

    // get the updated pose estimate (use this to transform the cloud before passing off to the mapping)
    auto pose_estimate {localizer_->pose()};
    auto T_estimate {conversion_utils::pose2TransformationMatrix(tf2::sophusToPose(pose_estimate))};

    // update mapping
    if (last_mapping_update_point_ == nullptr || 
        (last_mapping_update_point_ != nullptr &&
        getDisplacement(pose_estimate.translation(), *last_mapping_update_point_) > params_.mapping_threshold)
    ) {
        // use localization to update transform for cloud
        PointCloud::Ptr estimated_point_cloud (new PointCloud);
        pcl::transformPointCloud(*raw_point_cloud, *estimated_point_cloud, T_estimate);

        Pose3D robot_pose;
        robot_pose.position = pose_estimate.translation();
        robot_pose.orientation = pose_estimate.so3().unit_quaternion();

        mapper_->update(estimated_point_cloud, robot_pose);

        auto map {mapper_->map()};
        RCLCPP_INFO_STREAM(this->get_logger(), "Map has " << map->points.size() << " points.");

        last_mapping_update_point_ = std::make_unique<Eigen::Vector3d>(pose_estimate.translation());

        // if (map->points.size() > 0) {
        //     sensor_msgs::msg::PointCloud2 map_msg;
        //     pcl::toROSMsg(*map, map_msg);
        //     map_msg.header.frame_id = params_.fixed_frame;
        //     frame_publisher_->publish(map_msg);
        // }
    }

    auto tf {conversion_utils::transformationMatrix2Transform(T_estimate)};
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.transform = tf;
    tf_stamped.child_frame_id = params_.robot_frame;
    tf_stamped.header.frame_id = params_.fixed_frame;
    tf_broadcaster_->sendTransform(tf_stamped);

    counter_++;
}

std::optional<sensor_msgs::msg::PointCloud2> RosNode::transformPointCloud(const sensor_msgs::msg::PointCloud2 &msg, const std::string &frame)
{
    if (frame == msg.header.frame_id) {
        return msg;
    }
    try {
        sensor_msgs::msg::PointCloud2 transformed_msg;
        pcl_ros::transformPointCloud(frame, msg, transformed_msg, *tf_buffer_);
        return transformed_msg;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to transform cloud from frame '" << msg.header.frame_id << "' into frame '" << frame << "'.");
        return std::nullopt;
    }
}

} // namespace hylacomylus

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node {std::make_shared<hylacomylus::RosNode>(rclcpp::NodeOptions())};
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(hylacomylus::RosNode)
