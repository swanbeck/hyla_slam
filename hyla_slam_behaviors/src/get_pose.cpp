#include "hyla_slam_behaviors/get_pose.hpp"

namespace hyla_slam_behaviors {

GetPose::GetPose(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList GetPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("remote_hostname", "Hostname of the remote robot to call the service on. If not provided, calls the service in the local namespace."),
        BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", "Retrieved current PoseStamped estimate."),
    };
}

BT::NodeStatus GetPose::onStart()
{
    auto result {BT::NodeStatus::RUNNING};
    
    std::string service_handle {"hyla_slam/get_pose"};
    auto remote_hostname {getInput<std::string>("remote_hostname")};
    if (remote_hostname.has_value()) {
        service_client_ = node_->create_client<Trigger>("/" + remote_hostname.value() + "/" + service_handle);
    } else {
        service_client_ = node_->create_client<Trigger>(service_handle);
    }

    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for get pose service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto req {std::make_shared<Trigger::Request>()};

    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Get pose " << result << "...");

    return result;
}

BT::NodeStatus GetPose::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for get pose response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for get pose response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::SUCCESS};

            auto resp {request_future_->get()};
            setOutput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", std::make_shared<geometry_msgs::msg::PoseStamped>(resp->pose));

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Get pose responded! Returning " << return_status << "!");

            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void GetPose::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
