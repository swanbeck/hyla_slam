#include "hyla_slam/hylacomylus_ros_node.hpp"

namespace hylacomylus {

RosNode::RosNode(const rclcpp::NodeOptions &opts)
: rclcpp::Node("hyla_slam", opts), counter_(0)
{
    // TODO get parameters to fill out the configs

    // create the configs

    // create the two pipelines
    MappingConfig mapping_config;
    mapping_config.fixed_frame = "map";
    mapping_config.robot_frame = "os_sensor";
    mapping_config.odom_frame = "map";
    mapping_config.chunk_discretization = 1;
    mapping_config.chunk_load_dir = "/root/data/";
    mapping_config.half_side_length = 5.0;
    mapper_ = std::make_unique<Hylacomylus>(mapping_config);

    KissConfig localization_config;
    localization_config.voxel_size = 1.0;
    localization_config.max_range = 10.0;
    localization_config.min_range = 2.0;
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
        "/ouster/points",
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

    // odom_timer_ = this->create_wall_timer(1000ms, std::bind(&RosNode::publishOdometry, this));
    // map_timer_ = this->create_wall_timer(3000ms, std::bind(&RosNode::publishMap, this));

    // create required services

    // wait for starting location to be set before starting localization
    

    // for initial testing, create sub to lidar topic and just run it

}

void RosNode::handleNewFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    counter_++;
    RCLCPP_INFO_STREAM(this->get_logger(), "Inside handle frame : " << counter_ << ".");

    // update localization

    // transform cloud msg into pcl
    PointCloud::Ptr raw_point_cloud (new PointCloud);
    pcl::fromROSMsg(*msg, *raw_point_cloud);

    // transform cloud into fixed frame? or robot frame?
    auto previous_pose_estimate {tf2::sophusToPose(localizer_->pose())};
    auto previous_T_estimate {surface_repair_utils::transforms::pose2TransformationMatrix(previous_pose_estimate)};
    PointCloud::Ptr raw_robot_point_cloud (new PointCloud);
    pcl::transformPointCloud(*raw_point_cloud, *raw_robot_point_cloud, previous_T_estimate);

    RCLCPP_INFO_STREAM(this->get_logger(), "Previous pose: " << previous_pose_estimate.position.x << ", " << previous_pose_estimate.position.y << ", " << previous_pose_estimate.position.z << ".");

    // TODO only reset the map if we'll running hylacomylus on the update, otherwise let it update naturally
    if (counter_ % 50 == 0) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Setting reference for localization...");
        // set map cloud as reference
        auto map_msg {std::make_shared<sensor_msgs::msg::PointCloud2>()};
        auto map {mapper_->map()};

        // need to transform points into the sensor localization frame here I think?

        if (map->points.size() > 0) {
            PointCloud::Ptr map_robot_frame (new PointCloud);
            pcl::transformPointCloud(*map, *map_robot_frame, previous_T_estimate.inverse());

            pcl::toROSMsg(*map_robot_frame, *map_msg);
            std::vector<Eigen::Vector3d> map_reference {kiss_icp_ros::utils::PointCloud2ToEigen(map_msg)};
            localizer_->setMap(map_reference);

            map_msg->header.frame_id = "map";
            reference_publisher_->publish(*map_msg);
        }
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Registering frame...");

    // feed in frame (convert cloud to proper input type)
    localizer_->registerFrame(kiss_icp_ros::utils::PointCloud2ToEigen(msg));

    // get pose out
    auto updated_pose_estimate_sophus {localizer_->pose()};

    // convert sophus pose into geometry_msgs & Pose3D?
    auto updated_pose_estimate {tf2::sophusToPose(updated_pose_estimate_sophus)};
    auto updated_T_estimate{surface_repair_utils::transforms::pose2TransformationMatrix(updated_pose_estimate)};

    RCLCPP_INFO_STREAM(this->get_logger(), "Previous pose: " << updated_pose_estimate.position.x << ", " << updated_pose_estimate.position.y << ", " << updated_pose_estimate.position.z << ".");

    // update mapping
    if (counter_ % 50 == 0) {

        RCLCPP_INFO_STREAM(this->get_logger(), "Updating mapping...");

        // use it to transform cloud?
        PointCloud::Ptr robot_point_cloud (new PointCloud);
        // pcl::transformPointCloud(*raw_point_cloud, *robot_point_cloud, previous_T_estimate.inverse() * updated_T_estimate);
        pcl::transformPointCloud(*raw_point_cloud, *robot_point_cloud, updated_T_estimate);

        RCLCPP_INFO_STREAM(this->get_logger(), "Transformed cloud has " << robot_point_cloud->points.size() << " points.");

        Pose3D robot_pose;
        robot_pose.position.x() = updated_pose_estimate.position.x;
        robot_pose.position.y() = updated_pose_estimate.position.y;
        robot_pose.position.z() = updated_pose_estimate.position.z;
        robot_pose.orientation.w() = updated_pose_estimate.orientation.w;
        robot_pose.orientation.x() = updated_pose_estimate.orientation.x;
        robot_pose.orientation.y() = updated_pose_estimate.orientation.y;
        robot_pose.orientation.z() = updated_pose_estimate.orientation.z;

        RCLCPP_INFO_STREAM(this->get_logger(), "Indexing data...");
        mapper_->update(robot_point_cloud, robot_pose);

        // publish odometry and map for now
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing data...");
        auto map {mapper_->map()};
        RCLCPP_INFO_STREAM(this->get_logger(), "Map has " << map->points.size() << " points.");

        if (map->points.size() > 0) {
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(*map, output_cloud);
            output_cloud.header.frame_id = "map";

            // publish this cloud, also get and publish the map?
            // frame_publisher_->publish(output_cloud);
        }
    }

    // auto tf {surface_repair_utils::transforms::transformationMatrix2Transform(previous_T_estimate.inverse() * updated_T_estimate)};
    auto tf {surface_repair_utils::transforms::transformationMatrix2Transform(updated_T_estimate)};
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.transform = tf;
    tf_stamped.child_frame_id = "os_sensor";
    tf_stamped.header.frame_id = "map";
    tf_broadcaster_->sendTransform(tf_stamped);

    RCLCPP_INFO_STREAM(this->get_logger(), "Made it to the end!");
}

// TODO create wall timers for these (map should only publish when updated)
void RosNode::publishOdometry()
{

}

void RosNode::publishMap()
{

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
