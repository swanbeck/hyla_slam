#include "hyla_slam_behaviors/lookup_hashes.hpp"

namespace hyla_slam_behaviors {

LookupHashes::LookupHashes(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList LookupHashes::providedPorts()
{
    return {
        BT::InputPort<std::string>("remote_hostname", "Hostname of the remote robot to call the service on. If not provided, calls the service in the local namespace."),
        BT::InputPort<int>("location", "Location index to look up hashes for."),
        BT::OutputPort<std::shared_ptr<std::deque<std::string>>>("hashes", "Retrieved list of data hashes for the specified location."),
    };
}

BT::NodeStatus LookupHashes::onStart()
{
    auto result {BT::NodeStatus::RUNNING};

    std::string service_handle {"hyla_slam/lookup_hashes"};
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
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for lookup hashes service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto location_option {getInput<int>("location")};

    auto req {std::make_shared<Trigger::Request>()};
    if (location_option.has_value()) {
        req->location = location_option.value();
    }

    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Lookup hashes " << result << "...");

    return result;
}

BT::NodeStatus LookupHashes::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for lookup hashes response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for lookup hashes response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::SUCCESS};

            auto resp {request_future_->get()};
            setOutput("hashes", std::make_shared<std::deque<std::string>>(resp->hashes.begin(), resp->hashes.end()));

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Lookup hashes responded! Returning " << return_status << "!");

            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void LookupHashes::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
