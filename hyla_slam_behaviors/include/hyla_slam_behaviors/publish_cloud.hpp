#pragma once

#include <future>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <behaviortree_cpp/action_node.h>

namespace hyla_slam_behaviors {

class PublishCloud : public BT::SyncActionNode
{
public:
    PublishCloud(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;

}; // class PublishCloud

} // namespace hyla_slam_behaviors
