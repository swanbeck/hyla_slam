#include "hyla_slam_behaviors/set_localization_estimate.hpp"

namespace hyla_slam_behaviors {

SetLocalizationEstimate::SetLocalizationEstimate(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList SetLocalizationEstimate::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
}

BT::NodeStatus SetLocalizationEstimate::onStart()
{
    // read in targets
    auto pose_option {getInput<geometry_msgs::msg::PoseStamped>("pose")};

    auto result {BT::NodeStatus::RUNNING};
    service_client_ = node_->create_client<Trigger>("hyla_slam/set_localization_estimate");

    // add timeout to wait for the service
    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for set localization estimate service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    // construct a request
    auto req {std::make_shared<Trigger::Request>()};

    if (pose_option.has_value()) {
        req->pose = pose_option.value();
    } else {
        RCLCPP_INFO(node_->get_logger(), "No pose provided! Assuming Identity pose!");
        req->identity = true;
    }

    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Set localization estimate " << result << "...");

    return result;
}

BT::NodeStatus SetLocalizationEstimate::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for set localization estimate response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for set localization estimate response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::SUCCESS}; 
            // auto resp {request_future_->get()};
            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Set localization estimate responded! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void SetLocalizationEstimate::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
