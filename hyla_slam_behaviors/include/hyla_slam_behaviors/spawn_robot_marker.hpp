#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <visualization_msgs/msg/marker.hpp>

namespace hyla_slam_behaviors {

class SpawnRobotMarker : public BT::SyncActionNode
{
public:
    SpawnRobotMarker(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_;

}; // class SpawnRobotMarker

} // namespace hyla_slam_behaviors
