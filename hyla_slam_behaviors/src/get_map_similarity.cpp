#include "hyla_slam_behaviors/get_map_similarity.hpp"

namespace hyla_slam_behaviors {

GetMapSimilarity::GetMapSimilarity(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList GetMapSimilarity::providedPorts()
{
    return {
        BT::InputPort<double>("threshold"),
    };
}

BT::NodeStatus GetMapSimilarity::onStart()
{
    threshold_ = getInput<double>("threshold").value();

    auto result {BT::NodeStatus::RUNNING};
    service_client_ = node_->create_client<Trigger>("hyla_slam/get_map_similarity");

    // add timeout to wait for the service
    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for get map similarity service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    // construct a request
    auto req {std::make_shared<Trigger::Request>()};
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get map similarity " << result << "...");

    return result;
}

BT::NodeStatus GetMapSimilarity::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for get map similarity response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for get map similarity response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::FAILURE};

            auto resp {request_future_->get()};

            if (resp->score > threshold_) {
                return_status = BT::NodeStatus::SUCCESS;
            }

            request_future_ = std::nullopt;
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get map similarity responded (" << resp->score << ")! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void GetMapSimilarity::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
