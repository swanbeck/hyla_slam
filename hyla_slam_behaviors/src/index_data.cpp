#include "hyla_slam_behaviors/index_data.hpp"

namespace hyla_slam_behaviors {

IndexData::IndexData(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList IndexData::providedPorts()
{
    return {
        BT::InputPort<std::string>("remote_hostname", "Hostname of the remote robot to call the service on. If not provided, calls the service in the local namespace."),
        BT::InputPort<std::shared_ptr<sensor_msgs::msg::PointCloud2>>("point_cloud", "Point cloud data to index."),
        BT::InputPort<std::shared_ptr<geometry_msgs::msg::TransformStamped>>("local_transform", "Local transform to associate with the indexed data. Used only if global_transform is also provided."),
        BT::InputPort<std::shared_ptr<geometry_msgs::msg::TransformStamped>>("global_transform", "Global transform for the indexed data. If provided, skips the internal transform lookup."),
        BT::InputPort<bool>("unload_data", "If true, unloads data from active memory after indexing."),
        BT::OutputPort<double>("time_ms", "Time taken for the indexing operation in milliseconds."),
    };
}

BT::NodeStatus IndexData::onStart()
{
    auto result {BT::NodeStatus::RUNNING};
    
    auto cloud_option {getInput<std::shared_ptr<sensor_msgs::msg::PointCloud2>>("point_cloud")};
    auto local_transform_option {getInput<std::shared_ptr<geometry_msgs::msg::TransformStamped>>("local_transform")};
    auto global_transform_option {getInput<std::shared_ptr<geometry_msgs::msg::TransformStamped>>("global_transform")};
    auto cloud = cloud_option.value_or(std::make_shared<sensor_msgs::msg::PointCloud2>());
    auto unload_data_option {getInput<bool>("unload_data")};

    std::string service_handle {"hyla_slam/index_data"};
    auto remote_hostname {getInput<std::string>("remote_hostname")};
    if (remote_hostname.has_value()) {
        service_client_ = node_->create_client<Trigger>("/" + remote_hostname.value() + "/" + service_handle);
    } else {
        service_client_ = node_->create_client<Trigger>(service_handle);
    }

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
    req->cloud = *cloud;
    if (global_transform_option.has_value()) {
        req->lookup_transform = false;
        req->global_transform = *(global_transform_option.value());
        
        if (local_transform_option.has_value()) {
            req->local_transform = *(local_transform_option.value());
        }
    }

    if (unload_data_option.has_value()) {
        req->unload_data = unload_data_option.value();
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
            auto resp {request_future_->get()};
            request_future_ = std::nullopt;
            setOutput<double>("time_ms", resp->time_ms);
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
