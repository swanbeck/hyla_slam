#pragma once

#include <memory>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "hyla_kiss/hyla_kiss.hpp"
#include "hyla_kiss/kiss_icp_utils.hpp"
#include "hylacomylus/hylacomylus.hpp"
#include "hylacomylus/types.hpp"
#include "hyla_slam_parameters.hpp"
#include "hyla_slam_interfaces/srv/get_displacement.hpp"
#include "hyla_slam_interfaces/srv/get_map.hpp"
#include "hyla_slam_interfaces/srv/get_map_similarity.hpp"
#include "hyla_slam_interfaces/srv/index_data.hpp"
#include "hyla_slam_interfaces/srv/manage_local_storage.hpp"
#include "hyla_slam_interfaces/srv/set_pose.hpp"

using namespace boost::multiprecision;

namespace hyla_slam {

class BehaviorsNode : public rclcpp::Node
{
public:
    BehaviorsNode(const rclcpp::NodeOptions &opts);

    ~BehaviorsNode() {
        stopExecutors();
    }

    enum Capability { LOCALIZATION, MAPPING };

private:
    void manageLocalStorage(const std::shared_ptr<hyla_slam_interfaces::srv::ManageLocalStorage::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::ManageLocalStorage::Response> response);

    void enableLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void disableLocalization(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void unloadData(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void updateLocalization(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_msg);
    
    void getDisplacement(const Capability capability, std::shared_ptr<hyla_slam_interfaces::srv::GetDisplacement::Response> response);
    void getMap(const std::shared_ptr<hyla_slam_interfaces::srv::GetMap::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::GetMap::Response> response);
    void indexData(const std::shared_ptr<hyla_slam_interfaces::srv::IndexData::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::IndexData::Response> response);
    void updateLocalizationMap(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void setLocalizationEstimate(const std::shared_ptr<hyla_slam_interfaces::srv::SetPose::Request> request, std::shared_ptr<hyla_slam_interfaces::srv::SetPose::Response>);
    void getMapSimilarity(const std::shared_ptr<hyla_slam_interfaces::srv::GetMapSimilarity::Request>, std::shared_ptr<hyla_slam_interfaces::srv::GetMapSimilarity::Response> response);

    void startExecutors()
    {
        localization_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        localization_executor_->add_callback_group(localization_cb_group_, this->get_node_base_interface());
        service_executor_->add_callback_group(service_cb_group_, this->get_node_base_interface());

        localization_thread_ = std::thread([this]() { localization_executor_->spin(); });
        service_thread_ = std::thread([this]() { service_executor_->spin(); });
    }
    void stopExecutors()
    {
        if (localization_executor_ && service_executor_) {
            localization_executor_->cancel();
            service_executor_->cancel();
        }

        if (localization_thread_.joinable()) {
            localization_thread_.join();
        }

        if (service_thread_.joinable()) {
            service_thread_.join();
        }
    }

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> point_cloud_subscription_;

    std::shared_ptr<rclcpp::CallbackGroup> localization_cb_group_;
    std::shared_ptr<rclcpp::CallbackGroup> service_cb_group_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> localization_executor_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> service_executor_;

    std::thread localization_thread_;
    std::thread service_thread_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<hylacomylus::Hylacomylus> mapper_;
    std::unique_ptr<hyla_kiss::HylaKiss> localizer_;

    std::shared_ptr<hyla_slam::ParamListener> param_listener_;
    hyla_slam::Params params_;

    hylacomylus::MappingConfig mapping_config_;
    hyla_kiss::KissConfig localization_config_;

    bool localization_enabled_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> enable_localization_server_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> disable_localization_server_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> unload_data_server_;

    std::optional<Sophus::SE3d> localization_reference_pose_;
    std::optional<Sophus::SE3d> mapping_reference_pose_;

    std::set<uint256_t> latest_hashes_;
    std::set<uint256_t> localization_reference_hashes_;

    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::GetDisplacement>> get_localization_displacement_server_;
    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::GetDisplacement>> get_mapping_displacement_server_;

    std::shared_mutex mutex_;

    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::GetMap>> get_map_server_;
    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::GetMapSimilarity>> get_map_similarity_server_;
    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::IndexData>> index_data_server_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> update_localization_map_server_;
    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::SetPose>> set_localization_estimate_server_;

    std::shared_ptr<rclcpp::Service<hyla_slam_interfaces::srv::ManageLocalStorage>> manage_local_storage_server_;

    std::optional<std::set<uint256_t>> disk_hashes_ {std::nullopt};

}; // class BehaviorsNode

} // namespace hyla_slam
