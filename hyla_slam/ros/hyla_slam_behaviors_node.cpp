#include "hyla_slam_behaviors_node.hpp"

namespace hylacomylus {

BehaviorsNode::BehaviorsNode(const rclcpp::NodeOptions &opts)
: rclcpp::Node("hyla_slam", opts), localization_enabled_(false)
{
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

    localization_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // point cloud subscription (for localization)
    std::optional<rclcpp::QoS> sensor_data_qos {std::nullopt};
    if (params_.sensor_drivers_bridged) {
        sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    }
    auto localization_cb_options {rclcpp::SubscriptionOptions()};
    localization_cb_options.callback_group = localization_cb_group_;
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.point_cloud_topic,
        sensor_data_qos.value_or(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()),
        std::bind(&BehaviorsNode::updateLocalization, this, std::placeholders::_1),
        localization_cb_options
    );

    // services for behaviors
    enable_localization_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/enable_localization",
        std::bind(&BehaviorsNode::enableLocalization, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    disable_localization_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/disable_localization",
        std::bind(&BehaviorsNode::disableLocalization, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    unload_data_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/unload_data",
        std::bind(&BehaviorsNode::unloadData, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    get_localization_displacement_server_ = this->create_service<hyla_slam_interfaces::srv::GetDisplacement>(
        "~/get_localization_displacement",
        [this](const std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Request>, std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Response> response) {
            getDisplacement(Capability::LOCALIZATION, response);
        },
        rmw_qos_profile_default,
        service_cb_group_
    );
    get_mapping_displacement_server_ = this->create_service<hyla_slam_interfaces::srv::GetDisplacement>(
        "~/get_mapping_displacement",
        [this](const std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Request>, std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Response> response) {
            getDisplacement(Capability::MAPPING, response);
        },
        rmw_qos_profile_default,
        service_cb_group_
    );
    get_map_server_ = this->create_service<hyla_slam_interfaces::srv::GetMap>(
        "~/get_map",
        std::bind(&BehaviorsNode::getMap, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    index_data_server_ = this->create_service<hyla_slam_interfaces::srv::IndexData>(
        "~/index_data",
        std::bind(&BehaviorsNode::indexData, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    update_localization_map_server_ = this->create_service<std_srvs::srv::Trigger>(
        "~/update_localization_map",
        std::bind(&BehaviorsNode::updateLocalizationMap, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    set_localization_estimate_server_ = this->create_service<hyla_slam_interfaces::srv::SetPose>(
        "~/set_localization_estimate",
        std::bind(&BehaviorsNode::setLocalizationEstimate, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );

    startExecutors();
    RCLCPP_INFO(this->get_logger(), "Up and ready!");
}

void BehaviorsNode::setLocalizationEstimate(const std::shared_ptr<hyla_slam_interfaces::srv::SetPose::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::SetPose::Response>)
{
    if (!(request->pose.header.frame_id == params_.fixed_frame)) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Provided pose frame id (" << request->pose.header.frame_id << ") does match set fixed frame (" << params_.fixed_frame << "). Ignoring request...");
    }
    Sophus::SE3d pose(conversion_utils::pose2TransformationMatrix(request->pose.pose));
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        localizer_->setPose(pose);
    }
}

void BehaviorsNode::updateLocalizationMap(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // rescope map to current location
    Sophus::SE3d current_pose_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        current_pose_estimate = localizer_->pose();
    }
    Pose3D current_pose;
    current_pose.position = current_pose_estimate.translation();
    current_pose.orientation = current_pose_estimate.so3().unit_quaternion();

    PointCloud::Ptr dummy_cloud (new PointCloud);
    mapper_->update(dummy_cloud, current_pose);

    // get map from mapper
    auto map {mapper_->map()};

    if (!map->points.size() > 0) {
        response->success = false;
        response->message = "Map stored by mapper has 0 points!";
        return;
    }

    // convert it to msg
    auto map_msg {std::make_shared<sensor_msgs::msg::PointCloud2>()};
    pcl::toROSMsg(*map, *map_msg);

    // update localization map
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        localizer_->setMap(kiss_icp_ros::utils::PointCloud2ToEigen(map_msg));
    }

    response->success = true;
    response->message = "Localization map updated!";
    RCLCPP_INFO(this->get_logger(), "Localization map updated!");
}

void BehaviorsNode::indexData(const std::shared_ptr<hyla_slam_interfaces::srv::IndexData::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::IndexData::Response> response)
{
    // get current pose
    Sophus::SE3d current_pose_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        current_pose_estimate = localizer_->pose();
    }
    Pose3D current_pose;
    current_pose.position = current_pose_estimate.translation();
    current_pose.orientation = current_pose_estimate.so3().unit_quaternion();

    // transform cloud (using request transform if provided)
    PointCloud::Ptr input_point_cloud (new PointCloud);
    pcl::fromROSMsg(request->cloud, *input_point_cloud);
    
    PointCloud::Ptr transformed_point_cloud (new PointCloud);
    Eigen::Matrix4d tf = request->lookup_transform ? current_pose_estimate.matrix() : conversion_utils::transformStamped2TransformationMatrix(request->transform);
    pcl::transformPointCloud(*input_point_cloud, *transformed_point_cloud, tf);

    // index data in mapper
    mapper_->update(transformed_point_cloud, current_pose);
    
    // update mapping reference pose
    mapping_reference_pose_ = std::make_unique<Sophus::SE3d>(tf);

    RCLCPP_INFO_STREAM(this->get_logger(), "Data indexed to existing map! Updated map has " << mapper_->map()->points.size() << " points.");
}

void BehaviorsNode::getMap(const std::shared_ptr<hyla_slam_interfaces::srv::GetMap::Request>, std::shared_ptr<hyla_slam_interfaces::srv::GetMap::Response> response)
{
    // convert map to PC2 and return
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*(mapper_->map()), msg);
    msg.header.frame_id = params_.fixed_frame;
    response->map = msg;
    RCLCPP_INFO_STREAM(this->get_logger(), "Returning map of size " << mapper_->map()->size() << ".");
}

void BehaviorsNode::getDisplacement(const Capability capability, std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Response> response)
{
    // get reference pose
    Sophus::SE3d reference_pose;
    switch (capability) {
        case (Capability::LOCALIZATION): {
            if (localization_reference_pose_ == nullptr) { return; }
            reference_pose = *localization_reference_pose_;
            break;
        }
        case (Capability::MAPPING): {
            if (mapping_reference_pose_ == nullptr) { return; }
            reference_pose = *mapping_reference_pose_;
            break;
        }
        default: {
            return;
        }
    }

    // get current pose
    Sophus::SE3d current_pose;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        current_pose = localizer_->pose();
    }

    auto transform {current_pose.matrix() * reference_pose.matrix().inverse()};
    
    // compute linear displacement
    response->linear = transform.block<3,1>(0,3).norm();

    // compute angular displacement
    Eigen::AngleAxisd axis_angle(transform.block<3,3>(0,0));
    response->angular = axis_angle.angle();
}

void BehaviorsNode::updateLocalization(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_msg)
{
    // TODO
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (!localization_enabled_) {
            return;
        }
    }

    // transform cloud into localization frame
    auto transformed_msg {BehaviorsNode::transformPointCloud(*raw_msg, params_.robot_frame)};

    if (!transformed_msg.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform cloud into robot frame! No SLAM update performed.");
        return;
    }

    // register the frame
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg(std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_msg.value()));

    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        localizer_->registerFrame(kiss_icp_ros::utils::PointCloud2ToEigen(msg));
    }

    // get pose back out and publish tf
    Sophus::SE3d pose_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        pose_estimate = localizer_->pose();
    }

    auto tf {tf2::sophusToTransform(pose_estimate)};
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.transform = tf;
    tf_stamped.child_frame_id = params_.robot_frame;
    tf_stamped.header.frame_id = params_.fixed_frame;
    tf_broadcaster_->sendTransform(tf_stamped);
}

void BehaviorsNode::enableLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::string msg {};
    if (localization_enabled_ == true) {
        msg = "Localization already enabled!";
        RCLCPP_WARN(this->get_logger(), msg.c_str());
    } else {
        msg = "Localization enabled!";
        RCLCPP_INFO(this->get_logger(), msg.c_str());
    }
    localization_enabled_ = true;
    response->success = true;
    response->message = msg;
}

void BehaviorsNode::disableLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::string msg {};
    if (localization_enabled_ == false) {
        msg = "Localization already disabled.";
        RCLCPP_WARN(this->get_logger(), msg.c_str());
    } else {
        msg = "Localization disabled.";
        RCLCPP_INFO(this->get_logger(), msg.c_str());
    }
    localization_enabled_ = false;
    response->success = true;
    response->message = msg;
}

void BehaviorsNode::unloadData(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    mapper_->dumpMemoryData();
    std::string msg {"Unloaded data in memory to disk!"};
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    localization_reference_pose_ = nullptr;
    mapping_reference_pose_ = nullptr;
    response->success = true;
    response->message = msg;
}

std::optional<sensor_msgs::msg::PointCloud2> BehaviorsNode::transformPointCloud(const sensor_msgs::msg::PointCloud2 &msg, const std::string &frame)
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

    auto node {std::make_shared<hylacomylus::BehaviorsNode>(rclcpp::NodeOptions())};
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(hylacomylus::BehaviorsNode)
