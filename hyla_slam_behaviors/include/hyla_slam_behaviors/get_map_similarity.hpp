#pragma once

#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <hyla_slam_interfaces/srv/get_map_similarity.hpp>
#include <behaviortree_cpp/action_node.h>

namespace hyla_slam_behaviors {

class GetMapSimilarity : public BT::StatefulActionNode
{
public:
    using Trigger = hyla_slam_interfaces::srv::GetMapSimilarity;

    GetMapSimilarity(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Client<Trigger>> service_client_;
    std::optional<rclcpp::Client<Trigger>::FutureAndRequestId> request_future_;

    double threshold_;

}; // class GetMapSimilarity

} // namespace hyla_slam_behaviors
