#include "behaviors_node.hpp"

namespace hyla_slam {

BehaviorsNode::BehaviorsNode(const rclcpp::NodeOptions &opts)
: rclcpp::Node("hyla_slam", opts), localization_enabled_(false)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); // prevent warnings from pcl::conversions being printed constantly

    // handle parameters
    param_listener_ = std::make_shared<hyla_slam::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    // create the configs and the two pipelines
    auto mapping_params = params_.mapping;
    mapping_config_.data_dir = mapping_params.data_dir;
    mapping_config_.active_mapping = mapping_params.active_mapping;
    mapping_config_.chunk_discretization = mapping_params.chunk_discretization;
    mapping_config_.persist_recent_chunks = mapping_params.persist_recent_chunks;
    mapping_config_.scan_memory_horizon = mapping_params.scan_memory_horizon;
    mapping_config_.dense_map_radius = mapping_params.dense_map_radius;
    mapping_config_.sparse_map_radius = mapping_params.sparse_map_radius;
    mapping_config_.max_points_per_dense_chunk = mapping_params.max_points_per_dense_chunk;
    mapping_config_.max_points_per_sparse_chunk = mapping_params.max_points_per_sparse_chunk;
    mapping_config_.sparse_voxel_size = mapping_params.sparse_voxel_size;
    mapping_config_.save_dense_scans = mapping_params.save_dense_scans;
    mapping_config_.save_sparse_scans = mapping_params.save_sparse_scans;
    mapping_config_.maintain_dense_chunks = mapping_params.maintain_dense_chunks;
    mapping_config_.maintain_sparse_chunks = mapping_params.maintain_sparse_chunks;
    mapper_ = std::make_unique<hylacomylus::Hylacomylus>(mapping_config_);

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

    // create ROS stuff
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    localization_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // point cloud subscription (for localization)
    std::optional<rclcpp::QoS> sensor_data_qos {std::nullopt};
    if (params_.best_effort_qos) {
        sensor_data_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    }
    auto localization_cb_options {rclcpp::SubscriptionOptions()};
    localization_cb_options.callback_group = localization_cb_group_;
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.point_cloud_topic,
        sensor_data_qos.value_or(rclcpp::QoS(rclcpp::KeepLast(1)).reliable()),
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
    get_map_similarity_server_ = this->create_service<hyla_slam_interfaces::srv::GetMapSimilarity>(
        "~/get_map_similarity",
        std::bind(&BehaviorsNode::getMapSimilarity, this, std::placeholders::_1, std::placeholders::_2),
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

    manage_local_storage_server_ = this->create_service<hyla_slam_interfaces::srv::ManageLocalStorage>(
        "~/manage_local_storage",
        std::bind(&BehaviorsNode::manageLocalStorage, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    lookup_hashes_server_ = this->create_service<hyla_slam_interfaces::srv::LookupHashes>(
        "~/lookup_hashes",
        std::bind(&BehaviorsNode::lookupHashes, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );
    get_pose_server_ = this->create_service<hyla_slam_interfaces::srv::GetPose>(
        "~/get_pose",
        std::bind(&BehaviorsNode::getPoseEstimate, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default,
        service_cb_group_
    );

    startExecutors();
    RCLCPP_INFO(this->get_logger(), "Up and ready!");
}

void BehaviorsNode::lookupHashes(const std::shared_ptr<hyla_slam_interfaces::srv::LookupHashes::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::LookupHashes::Response> response)
{
    std::set<hash256_t> hashes;

    if (static_cast<int>(request->location) == static_cast<int>(hyla_slam_interfaces::srv::LookupHashes::Request::DISK)) {
        hashes = mapper_->lookupDiskHashes();
    } else if (static_cast<int>(request->location) == static_cast<int>(hyla_slam_interfaces::srv::LookupHashes::Request::MEMORY)) {
        hashes = mapper_->lookupMemoryHashes();
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid hash location " << static_cast<int>(request->location) << " specified!");
        return;
    }

    std::vector<std::string> hash_strings;
    for (const auto &hash : hashes) {
        hash_strings.push_back(to_hex_string(hash));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Returning " << hash_strings.size() << " hashes.");
    response->hashes = hash_strings;
}

void BehaviorsNode::manageLocalStorage(const std::shared_ptr<hyla_slam_interfaces::srv::ManageLocalStorage::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::ManageLocalStorage::Response> response)
{
    const auto &pose_msg = request->pose.pose;
    Eigen::Quaterniond orientation(pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z);
    Eigen::Vector3d position(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    auto pose_estimate {Sophus::SE3d(orientation, position)};

    std::set<hash256_t> search_hashes;
    for (const auto &hash_str : request->search_hashes) {
        hash256_t hash {hylacomylus::from_hex_string(hash_str)};
        search_hashes.insert(hash);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Managing local storage with " << search_hashes.size() << " hashes.");

    auto [similarity, disk_not_local, local_not_disk] = mapper_->manageDisk(pose_estimate, request->similarity_threshold, request->radius, std::nullopt, search_hashes);

    response->similarity = similarity;
    response->load_files = local_not_disk;
    response->unload_files = disk_not_local;
}

void BehaviorsNode::getPoseEstimate(std::shared_ptr<hyla_slam_interfaces::srv::GetPose::Request>, std::shared_ptr<hyla_slam_interfaces::srv::GetPose::Response> response)
{
    Sophus::SE3d map_lidar_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        map_lidar_estimate = localizer_->pose();
    }

    // get the proper transform between cloud and fixed frame
    geometry_msgs::msg::TransformStamped robot_lidar_transform;
    try {
        robot_lidar_transform = tf_buffer_->lookupTransform(
            params_.localization_frame,
            params_.robot_frame,
            tf2::TimePointZero
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", params_.localization_frame.c_str(), params_.robot_frame.c_str(), ex.what());
        return;
    }

    // use it to back out an estimate for the transform of the robot frame WRT to the fixed frame
    auto robot_lidar_estimate {tf2::transformToSophus(robot_lidar_transform)};
    Sophus::SE3d map_robot_estimate {map_lidar_estimate * robot_lidar_estimate.inverse()};

    // now convert transform to pose and set
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = params_.fixed_frame;
    Eigen::Quaterniond quat(map_robot_estimate.rotationMatrix());
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.position.x = map_robot_estimate.translation().x();
    pose_msg.pose.position.y = map_robot_estimate.translation().y();
    pose_msg.pose.position.z = map_robot_estimate.translation().z();

    response->pose = pose_msg;
}

void BehaviorsNode::setLocalizationEstimate(const std::shared_ptr<hyla_slam_interfaces::srv::SetPose::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::SetPose::Response>)
{
    auto start {std::chrono::high_resolution_clock::now()};

    geometry_msgs::msg::PoseStamped map_robot_estimate;
    if (request->identity == true) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Request to set localization estimate recieved with identity field set true! Using identity transform for update.");
    }
    else if (!(request->pose.header.frame_id == params_.fixed_frame)) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Provided pose frame id (" << request->pose.header.frame_id << ") does match set fixed frame (" << params_.fixed_frame << "). Ignoring request...");
        return;
    } else {
        map_robot_estimate = request->pose;
    }

    // lookup transform between LiDAR sensor frame and robot frame
    geometry_msgs::msg::TransformStamped robot_lidar_transform;
    try {
        robot_lidar_transform = tf_buffer_->lookupTransform(
            params_.localization_frame,
            params_.robot_frame,
            tf2::TimePointZero
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", params_.localization_frame.c_str(), params_.robot_frame.c_str(), ex.what());
        return;
    }

    // convert to sophus poses
    Sophus::SE3d map_robot_pose {tf2::poseToSophus(map_robot_estimate)};
    Sophus::SE3d robot_lidar_pose(tf2::transformToSophus(robot_lidar_transform));

    // compute transform and set in localizer
    Sophus::SE3d map_lidar_pose {map_robot_pose * robot_lidar_pose};
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        localizer_->setPose(map_lidar_pose);
        auto pose = localizer_->pose();
    }

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "setLocalizationEstimate: " << elapsed.count() << "ms");
}

// TODO
void BehaviorsNode::updateLocalizationMap(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto start {std::chrono::high_resolution_clock::now()};

    // rescope map to current location
    Sophus::SE3d map_lidar_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        map_lidar_estimate = localizer_->pose();
    }

    // hylacomylus::PointCloud::Ptr dummy_cloud (new hylacomylus::PointCloud);
    // localization_reference_hashes_ = mapper_->update(dummy_cloud, map_lidar_estimate);
    localization_reference_pose_ = map_lidar_estimate;

    // get map from mapper
    auto map {mapper_->sparseMap(map_lidar_estimate)};

    if (!map->points.size() > 0) {
        std::string msg {"Map stored by mapper has 0 points!"};
        RCLCPP_WARN_STREAM(this->get_logger(), msg.c_str());
        response->success = true;
        response->message = msg;
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

    std::string msg {"Localization map updated!"};
    response->success = true;
    response->message = msg;
    RCLCPP_INFO(this->get_logger(), msg.c_str());

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "updateLocalizationMap: " << elapsed.count() << "ms");
}

void BehaviorsNode::indexData(const std::shared_ptr<hyla_slam_interfaces::srv::IndexData::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::IndexData::Response> response)
{
    auto start {std::chrono::high_resolution_clock::now()};

    // get current pose
    Sophus::SE3d map_lidar_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        map_lidar_estimate = localizer_->pose();
    }

    // get the proper transform between cloud and fixed frame
    Sophus::SE3d transform;
    if (request->lookup_transform == true) {
        geometry_msgs::msg::TransformStamped robot_lidar_transform;
        try {
            robot_lidar_transform = tf_buffer_->lookupTransform(
                request->cloud.header.frame_id,
                params_.fixed_frame,
                tf2::TimePointZero
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", params_.localization_frame.c_str(), params_.robot_frame.c_str(), ex.what());
            return;
        }
        transform = tf2::transformToSophus(robot_lidar_transform);
    } else {
        transform = (tf2::transformToSophus(request->local_transform) * tf2::transformToSophus(request->global_transform)).inverse();
    }

    // update the mapper
    hylacomylus::PointCloud::Ptr point_cloud (new hylacomylus::PointCloud);
    pcl::fromROSMsg(request->cloud, *point_cloud);
    mapper_->update(point_cloud, transform);
    
    // update mapping reference pose
    mapping_reference_pose_ = transform;

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "indexData: " << elapsed.count() << "ms");
}

// TODO consider relocating this
void BehaviorsNode::getMapSimilarity(const std::shared_ptr<hyla_slam_interfaces::srv::GetMapSimilarity::Request>, std::shared_ptr<hyla_slam_interfaces::srv::GetMapSimilarity::Response> response)
{
    if (latest_hashes_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot compute similarity with an empty latest hash set! Index new data to the map to enable this.");
        return;
    }

    // compute Jaccard Similarity between sets
    auto jaccard_similarity = [](const std::set<hash256_t> &s1, const std::set<hash256_t> &s2) -> double {
        std::set<hash256_t> intersection_set;
        std::set<hash256_t> union_set;

        std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(intersection_set, intersection_set.begin()));
        std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(union_set, union_set.begin()));

        if (union_set.empty()) { return 1.0; }

        return static_cast<double>(intersection_set.size()) / union_set.size();
    };

    if (localization_reference_hashes_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Localization reference hashes are empty!");
    }

    double similarity {jaccard_similarity(latest_hashes_, localization_reference_hashes_)};
    response->score = similarity;
}

void BehaviorsNode::getMap(const std::shared_ptr<hyla_slam_interfaces::srv::GetMap::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::GetMap::Response> response)
{
    auto start {std::chrono::high_resolution_clock::now()};

    // rescope map
    Sophus::SE3d current_pose;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        current_pose = localizer_->pose();
    }

    hylacomylus::PointCloud::Ptr dummy_cloud (new hylacomylus::PointCloud);

    // compute the transform between the last mapping reference pose and the current pose
    std::optional<Sophus::SE3d> projected_pose {std::nullopt};
    if (mapping_reference_pose_.has_value()) {
        Sophus::SE3d delta {mapping_reference_pose_.value().inverse() * current_pose};

        // project the current_pose forward
        projected_pose = current_pose * delta;
    }
    
    // TODO restore this once basic functionality works
    auto map {request->dense ? mapper_->denseMap(current_pose, request->radius, projected_pose) : mapper_->sparseMap(current_pose, request->radius, projected_pose)};

    // convert map to PC2 and return
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = params_.fixed_frame;
    response->map = msg;
    RCLCPP_INFO_STREAM(this->get_logger(), "Returning map of size " << map->size() << ".");

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "getMap: " << elapsed.count() << "ms");
}

void BehaviorsNode::getDisplacement(const Capability capability, std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Response> response)
{
    // get reference pose
    Sophus::SE3d reference_pose;
    switch (capability) {
        case (Capability::LOCALIZATION): {
            if (!(localization_reference_pose_.has_value())) { return; }
            reference_pose = localization_reference_pose_.value();
            break;
        }
        case (Capability::MAPPING): {
            if (!(mapping_reference_pose_.has_value())) { return; }
            reference_pose = mapping_reference_pose_.value();
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
    auto start {std::chrono::high_resolution_clock::now()};
    
    // check to make sure data is expressed in the proper frame
    if (raw_msg->header.frame_id != params_.localization_frame) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Incoming sensor data is in frame " << raw_msg->header.frame_id << " but localization_frame is " << params_.localization_frame << "; these should match. No localization update will be performed.");
        return;
    }

    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        if (!localization_enabled_) {
            return;
        }
    }

    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        localizer_->registerFrame(kiss_icp_ros::utils::PointCloud2ToEigen(raw_msg));
    }

    Sophus::SE3d pose_estimate;
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        pose_estimate = localizer_->pose();
    }

    // lookup transform between sensor frame and robot
    geometry_msgs::msg::TransformStamped robot_lidar_transform;
    try {
        robot_lidar_transform = tf_buffer_->lookupTransform(
            params_.localization_frame,
            params_.robot_frame,
            tf2::TimePointZero
        );
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", params_.localization_frame.c_str(), params_.robot_frame.c_str(), ex.what());
        return;
    }

    // use it to back out an estimate for the transform of the robot frame WRT to the fixed frame
    auto robot_lidar_pose {tf2::transformToSophus(robot_lidar_transform)};
    Sophus::SE3d map_robot_pose {pose_estimate * robot_lidar_pose.inverse()};

    // broadcast it
    geometry_msgs::msg::TransformStamped map_robot_tf;
    map_robot_tf.transform = tf2::sophusToTransform(map_robot_pose);
    map_robot_tf.header.frame_id = params_.fixed_frame;
    map_robot_tf.child_frame_id = params_.robot_frame;
    map_robot_tf.header.stamp = raw_msg->header.stamp;
    tf_broadcaster_->sendTransform(map_robot_tf);

    auto end {std::chrono::high_resolution_clock::now()};
    std::chrono::duration<double, std::milli> elapsed {end - start};
    RCLCPP_DEBUG_STREAM(this->get_logger(), "updateLocalization: " << elapsed.count() << "ms");
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
    mapper_->unloadData();
    std::string msg {"Unloaded data in memory to disk!"};
    RCLCPP_INFO(this->get_logger(), msg.c_str());
    localization_reference_pose_ = std::nullopt;
    mapping_reference_pose_ = std::nullopt;
    response->success = true;
    response->message = msg;
}

} // namespace hyla_slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node {std::make_shared<hyla_slam::BehaviorsNode>(rclcpp::NodeOptions())};
    rclcpp::spin(node);

    rclcpp::shutdown();
    
    return 0;
}

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(hylacomylus::BehaviorsNode)
