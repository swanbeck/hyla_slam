#pragma once

#include <string>
#include <memory>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "surface_repair_common/utils.hpp"

#include "hyla_slam/hyla_kiss.hpp"
#include "hyla_slam/hylacomylus.hpp"
#include "hyla_slam/types.hpp"
#include "hyla_slam/kiss_icp_utils.hpp"

namespace hylacomylus {

class RosNode : public rclcpp::Node
{
public:
    RosNode(const rclcpp::NodeOptions &opts);

private:
    void handleNewFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    void enableSLAM(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void disableSLAM(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void unloadData(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    double getDisplacement(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2);

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<rclcpp::TimerBase> odom_timer_;
    std::shared_ptr<rclcpp::TimerBase> map_timer_;

    std::unique_ptr<HylaKiss> localizer_;
    std::unique_ptr<Hylacomylus> mapper_;

    std::shared_ptr<sensor_msgs::msg::PointCloud2> map_msg_;

    int counter_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> subscription_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> reference_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> frame_publisher_;

    // TODO how to localize not starting from origin? need to have reasonably accurate estimate of pose and pass it in to mapper_ and localizer_ before loading starting map and/or trying to localize
    std::optional<geometry_msgs::msg::PoseStamped> initial_robot_pose_;

    // first gen ros2 services
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> enable_slam_server_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> disable_slam_server_;
    bool slam_enabled_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> unload_data_server_;

    double localization_reference_threshold_;
    double mapping_update_threshold_;
    std::unique_ptr<Eigen::Vector3d> last_localization_update_point_;
    std::unique_ptr<Eigen::Vector3d> last_mapping_update_point_;

}; // class RosNode

} // namespace hylacomylus
