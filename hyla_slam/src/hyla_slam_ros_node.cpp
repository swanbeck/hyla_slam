#include "hyla_slam/hyla_slam_ros_node.hpp"

namespace hylacomylus {

RosNode::RosNode(const rclcpp::NodeOptions &opts)
: rclcpp::Node("hyla_slam", opts), counter_(0), slam_enabled_(false)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); // prevent warnings from pcl::conversions being printed constantly

    auto declareParameter = [this](std::string name, auto type_val) {
        try {
            return this->declare_parameter<decltype(type_val)>(name);
        } catch(rclcpp::ParameterTypeException& e) {
            RCLCPP_WARN(this->get_logger(), "Parameter error for parameter \"%s\": %s", name.c_str(), e.what());
            exit(1);
        }
    };

    localization_reference_threshold_ = declareParameter("localization_threshold", double{});
    mapping_update_threshold_ = declareParameter("mapping_threshold", double{});
    auto point_cloud_topic {declareParameter("point_cloud_topic", std::string{})};

    // TODO get parameters to fill out the configs
    // create the configs and the two pipelines
    MappingConfig mapping_config;
    mapping_config.fixed_frame = "map";
    mapping_config.robot_frame = "os_sensor";
    mapping_config.odom_frame = "map";
    mapping_config.chunk_discretization = 1;
    mapping_config.chunk_load_dir = "/root/data/";
    mapping_config.half_side_length = 5.0;
    mapper_ = std::make_unique<Hylacomylus>(mapping_config);

    KissConfig localization_config;
    localization_config.voxel_size = 0.5;
    localization_config.max_range = 10.0;
    localization_config.min_range = 1.0;
    localization_config.max_points_per_voxel = 20;
    localization_config.min_motion_th = 0.1;
    localization_config.initial_threshold = 2.0;
    localization_config.max_num_iterations = 500;
    localization_config.convergence_criterion = 0.0001;
    localization_config.max_num_threads = 0;
    localization_config.deskew = false;
    localizer_ = std::make_unique<HylaKiss>(localization_config);

    // create ROS stuff
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
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
    response->success = true;
}

double RosNode::getDisplacement(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    return (p1 - p2).norm();
}

void RosNode::handleNewFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    if (!slam_enabled_) { return; }

    RCLCPP_INFO_STREAM(this->get_logger(), "Inside handle frame : " << counter_ << ".");

    // TODO transform cloud into "localization frame" (like base_link) if it is not already
    PointCloud::Ptr raw_point_cloud (new PointCloud);
    pcl::fromROSMsg(*msg, *raw_point_cloud);

    // update cloud for localization
    auto pose_prior = localizer_->pose();
    if (last_localization_update_point_ == nullptr || 
        (last_localization_update_point_ != nullptr &&
        getDisplacement(pose_prior.translation(), *last_localization_update_point_) > localization_reference_threshold_)
    ) {
        auto map {mapper_->map()};
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting map in localizer (map has " << map->points.size() << " points).");
        // TODO
        if (map->points.size() > 0) {
            auto map_msg {std::make_shared<sensor_msgs::msg::PointCloud2>()};
            pcl::toROSMsg(*map, *map_msg);
            localizer_->setMap(kiss_icp_ros::utils::PointCloud2ToEigen(map_msg));

            last_localization_update_point_ = std::make_unique<Eigen::Vector3d>(pose_prior.translation());

            // map_msg->header.frame_id = "map";
            // reference_publisher_->publish(*map_msg);
        }
    }

    // register the frame
    localizer_->registerFrame(kiss_icp_ros::utils::PointCloud2ToEigen(msg));

    // get the updated pose estimate (use this to transform the cloud before passing off to the mapping)
    auto pose_estimate {localizer_->pose()};
    auto T_estimate {surface_repair_utils::transforms::pose2TransformationMatrix(tf2::sophusToPose(pose_estimate))};

    // update mapping
    if (last_mapping_update_point_ == nullptr || 
        (last_mapping_update_point_ != nullptr &&
        getDisplacement(pose_estimate.translation(), *last_mapping_update_point_) > mapping_update_threshold_)
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
        //     map_msg.header.frame_id = "map";
        //     frame_publisher_->publish(map_msg);
        // }
    }

    auto tf {surface_repair_utils::transforms::transformationMatrix2Transform(T_estimate)};
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.transform = tf;
    tf_stamped.child_frame_id = "os_sensor";
    tf_stamped.header.frame_id = "map";
    tf_broadcaster_->sendTransform(tf_stamped);

    counter_++;
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
