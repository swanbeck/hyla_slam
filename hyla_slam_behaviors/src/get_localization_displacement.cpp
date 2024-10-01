#include "hyla_slam_behaviors/get_localization_displacement.hpp"

namespace hyla_slam_behaviors {

GetLocalizationDisplacement::GetLocalizationDisplacement(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList GetLocalizationDisplacement::providedPorts()
{
    return {
        BT::InputPort<double>("linear_target"),
        BT::InputPort<double>("angular_target")
    };
}

BT::NodeStatus GetLocalizationDisplacement::onStart()
{
    // read in targets
    auto linear_target_option {getInput<double>("linear_target")};
    auto angular_target_option {getInput<double>("angular_target")};
    linear_target_ = linear_target_option.value();
    angular_target_ = angular_target_option.value();

    auto result {BT::NodeStatus::RUNNING};
    service_client_ = node_->create_client<Trigger>("hyla_slam/get_localization_displacement");

    // add timeout to wait for the service
    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for get localization displacement service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    // construct a request
    auto req {std::make_shared<Trigger::Request>()};
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Get localization displacement " << result << "...");

    return result;
}

BT::NodeStatus GetLocalizationDisplacement::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for get localization displacement response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for get localization displacement response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::FAILURE};

            auto resp {request_future_->get()};

            if (resp->linear < 0.0 || resp->angular < 0.0) {
                RCLCPP_WARN_STREAM(node_->get_logger(), "Linear or angular displacement is not initialized!");
            } else if (resp->linear > linear_target_ || resp->angular > angular_target_) {
                return_status = BT::NodeStatus::SUCCESS;
            }

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Get localization displacement responded (" << resp->linear << ", " << resp->angular << ")! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void GetLocalizationDisplacement::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
