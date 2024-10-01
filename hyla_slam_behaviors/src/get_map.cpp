#include "hyla_slam_behaviors/get_map.hpp"

namespace hyla_slam_behaviors {

GetMap::GetMap(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList GetMap::providedPorts()
{
    return {
        BT::OutputPort<sensor_msgs::msg::PointCloud2>("map")
    };
}

BT::NodeStatus GetMap::onStart()
{
    auto result {BT::NodeStatus::RUNNING};
    service_client_ = node_->create_client<Trigger>("get_map");

    // add timeout to wait for the service
    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for get map service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    // construct a request
    auto req {std::make_shared<Trigger::Request>()};
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Get map " << result << "...");

    return result;
}

BT::NodeStatus GetMap::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for get map response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for get map response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::SUCCESS};

            auto resp {request_future_->get()};
            setOutput("map", resp->map);

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Get map responded! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void GetMap::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
