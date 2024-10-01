#include "hyla_slam_behaviors/index_data.hpp"

namespace hyla_slam_behaviors {

IndexData::IndexData(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList IndexData::providedPorts()
{
    return {
        BT::InputPort<sensor_msgs::msg::PointCloud2>("point_cloud"),
        BT::InputPort<geometry_msgs::msg::TransformStamped>("transform")
    };
}

BT::NodeStatus IndexData::onStart()
{
    // read in targets
    auto cloud_option {getInput<sensor_msgs::msg::PointCloud2>("point_cloud")};
    auto transform_option {getInput<geometry_msgs::msg::TransformStamped>("transform")};
    auto cloud = cloud_option.value();

    auto result {BT::NodeStatus::RUNNING};
    service_client_ = node_->create_client<Trigger>("index_data");

    // add timeout to wait for the service
    std::chrono::milliseconds timeout(1000);
    auto start_time {std::chrono::steady_clock::now()};
    while (!service_client_->wait_for_service(std::chrono::milliseconds(10))) {
        if (std::chrono::steady_clock::now() - start_time > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for index data estimate service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    // construct a request
    auto req {std::make_shared<Trigger::Request>()};
    req->cloud = cloud;
    if (transform_option.has_value()) {
        req->lookup_transform = false;
        req->transform = transform_option.value();
    }
    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Index data " << result << "...");

    return result;
}

BT::NodeStatus IndexData::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for index data response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for index data response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::SUCCESS}; 
            // auto resp {request_future_->get()};
            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Index data responded! Returning " << return_status << "!");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void IndexData::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
