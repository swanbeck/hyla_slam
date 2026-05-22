#include "hyla_slam_behaviors/get_map.hpp"

namespace hyla_slam_behaviors {

GetMap::GetMap(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList GetMap::providedPorts()
{
    return {
        BT::InputPort<std::string>("remote_hostname", "Hostname of the remote robot to call the service on. If not provided, calls the service in the local namespace."),
        BT::InputPort<bool>("all_memory_data", false, "If true, returns all in-memory data rather than a radius-filtered map."),
        BT::InputPort<bool>("dense", true, "If true, returns the dense (high-resolution) map."),
        BT::InputPort<bool>("project", true, "If true, projects the map before returning."),
        BT::InputPort<double>("radius", 10.0, "Radius in meters for filtering the returned map."),
        BT::OutputPort<std::shared_ptr<sensor_msgs::msg::PointCloud2>>("map", "Retrieved PointCloud2 map."),
    };
}

BT::NodeStatus GetMap::onStart()
{
    auto result {BT::NodeStatus::RUNNING};

    std::string service_handle {"hyla_slam/get_map"};
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
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for get map service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto req {std::make_shared<Trigger::Request>()};
    req->dense = getInput<bool>("dense").value_or(true);
    req->project = getInput<bool>("project").value_or(true);
    req->radius = getInput<double>("radius").value_or(10.0);
    req->all_memory_data = getInput<bool>("all_memory_data").value_or(false);

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
            setOutput<std::shared_ptr<sensor_msgs::msg::PointCloud2>>("map", std::make_shared<sensor_msgs::msg::PointCloud2>(resp->map));

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
