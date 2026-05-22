#include "hyla_slam_behaviors/get_mapping_displacement.hpp"

namespace hyla_slam_behaviors {

GetMappingDisplacement::GetMappingDisplacement(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList GetMappingDisplacement::providedPorts()
{
    return {
        BT::InputPort<std::string>("remote_hostname", "Hostname of the remote robot to call the service on. If not provided, calls the service in the local namespace."),
        BT::InputPort<double>("linear_target", "Linear displacement threshold in meters. Returns SUCCESS if exceeded. Disabled if negative."),
        BT::InputPort<double>("angular_target", "Angular displacement threshold in radians. Returns SUCCESS if exceeded. Disabled if negative."),
    };
}

BT::NodeStatus GetMappingDisplacement::onStart()
{
    auto result {BT::NodeStatus::RUNNING};

    auto linear_target_option {getInput<double>("linear_target")};
    auto angular_target_option {getInput<double>("angular_target")};
    linear_target_ = linear_target_option.value_or(-1.0);
    angular_target_ = angular_target_option.value_or(-1.0);

    std::string service_handle {"hyla_slam/get_mapping_displacement"};
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
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for get mapping displacement service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto req {std::make_shared<Trigger::Request>()};
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get mapping displacement " << result << "...");

    return result;
}

BT::NodeStatus GetMappingDisplacement::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for get mapping displacement response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for get mapping displacement response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::FAILURE};

            auto resp {request_future_->get()};

            if (resp->linear < 0.0 || resp->angular < 0.0) {
                RCLCPP_WARN(node_->get_logger(), "Linear or angular displacement is not initialized!");
            } else if (linear_target_ > 0.0 && resp->linear > linear_target_) {
                return_status = BT::NodeStatus::SUCCESS;
            } else if (angular_target_ > 0.0 && resp->angular > angular_target_) {
                return_status = BT::NodeStatus::SUCCESS;
            }

            request_future_ = std::nullopt;
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get mapping displacement responded (" << resp->linear << ", " << resp->angular << ")! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void GetMappingDisplacement::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
