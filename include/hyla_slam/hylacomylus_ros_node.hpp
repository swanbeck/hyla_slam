#pragma once

#include <string>
#include <memory>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "surface_repair_common/utils.hpp"

#include "hyla_slam/hyla_kiss.hpp"
#include "hyla_slam/hylacomylus.hpp"
#include "hyla_slam/types.hpp"
#include "hyla_slam/Utils.hpp"

namespace hylacomylus {

class RosNode : public rclcpp::Node
{
public:
    RosNode(const rclcpp::NodeOptions &opts);

private:
    void handleNewFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    void handleNewFrame2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    void publishOdometry();

    void publishMap();

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

    std::optional<geometry_msgs::msg::PoseStamped> initial_robot_pose_;

}; // class RosNode

} // namespace hylacomylus
