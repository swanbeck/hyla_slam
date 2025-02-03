#include "hyla_slam_behaviors/update_localization_map.hpp"

namespace hyla_slam_behaviors {

UpdateLocalizationMap::UpdateLocalizationMap(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList UpdateLocalizationMap::providedPorts()
{
    return {};
}

BT::NodeStatus UpdateLocalizationMap::onStart()
{
    auto result {BT::NodeStatus::RUNNING};
    service_client_ = node_->create_client<Trigger>("hyla_slam/update_localization_map");

    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for update localization map service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto req {std::make_shared<Trigger::Request>()};
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Update localization map " << result << "...");

    return result;
}

BT::NodeStatus UpdateLocalizationMap::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for update localization map response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for update localization map response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto resp {request_future_->get()};

            auto return_status {BT::NodeStatus::FAILURE};
            if (resp->success == true) {
                return_status = BT::NodeStatus::SUCCESS;
            }

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Update localization map responded! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void UpdateLocalizationMap::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
