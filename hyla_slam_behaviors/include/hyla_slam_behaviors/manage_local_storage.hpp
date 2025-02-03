#pragma once

#include <future>
#include <memory>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <hyla_slam_interfaces/srv/manage_local_storage.hpp>
#include <behaviortree_cpp/action_node.h>

namespace hyla_slam_behaviors {

class ManageLocalStorage : public BT::StatefulActionNode
{
public:
    using Trigger = hyla_slam_interfaces::srv::ManageLocalStorage;

    ManageLocalStorage(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Client<Trigger>> service_client_;
    std::optional<rclcpp::Client<Trigger>::FutureAndRequestId> request_future_;

}; // class ManageLocalStorage

} // namespace hyla_slam_behaviors
