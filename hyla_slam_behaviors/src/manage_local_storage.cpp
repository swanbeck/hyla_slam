#include "hyla_slam_behaviors/manage_local_storage.hpp"

namespace hyla_slam_behaviors {

ManageLocalStorage::ManageLocalStorage(const std::string name, const BT::NodeConfig &config)
: BT::StatefulActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{}

BT::PortsList ManageLocalStorage::providedPorts()
{
    return {
        BT::InputPort<std::string>("remote_hostname", "Hostname of the remote robot to call the service on. If not provided, calls the service in the local namespace."),
        BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", "Current pose used for proximity-based storage management."),
        BT::InputPort<double>("similarity_threshold", "Map similarity threshold below which data may be swapped."),
        BT::InputPort<double>("radius", "Radius in meters used to determine relevant nearby data."),
        BT::InputPort<std::shared_ptr<std::deque<std::string>>>("search_hashes", "Optional list of hashes to consider when determining storage changes."),
        BT::OutputPort<std::shared_ptr<std::deque<std::string>>>("load_files", "List of data file hashes to load into active memory."),
        BT::OutputPort<std::shared_ptr<std::deque<std::string>>>("unload_files", "List of data file hashes to unload from active memory."),
    };
}

BT::NodeStatus ManageLocalStorage::onStart()
{
    auto result {BT::NodeStatus::RUNNING};
    
    auto pose_option {getInput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose")};
    auto threshold {getInput<double>("similarity_threshold").value()};
    auto radius {getInput<double>("radius").value()};

    std::string service_handle {"hyla_slam/manage_local_storage"};
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
            RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for manage local storage service!");
            return BT::NodeStatus::FAILURE;
        }
    }

    auto req {std::make_shared<Trigger::Request>()};
    req->similarity_threshold = threshold;
    req->radius = radius;

    if (pose_option.has_value()) {
        req->pose = *(pose_option.value());
    }

    auto search_hashes_option {getInput<std::shared_ptr<std::deque<std::string>>>("search_hashes")};
    if (search_hashes_option.has_value()) {
        req->search_hashes = std::vector<std::string>(search_hashes_option.value()->begin(), search_hashes_option.value()->end());
    }

    request_future_ = service_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Manage local storage " << result << "...");

    return result;
}

BT::NodeStatus ManageLocalStorage::onRunning()
{
    auto response {rclcpp::spin_until_future_complete(node_->get_node_base_interface(), request_future_.value(), std::chrono::milliseconds(5))};

    switch(response) {
        case rclcpp::FutureReturnCode::TIMEOUT: {
            RCLCPP_DEBUG(node_->get_logger(), "Timed waiting for manage local storage response! Still running...");
            return BT::NodeStatus::RUNNING;
        }
        case rclcpp::FutureReturnCode::INTERRUPTED: {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted waiting for manage local storage response. Aborting...");
            return BT::NodeStatus::FAILURE;
        }
        case rclcpp::FutureReturnCode::SUCCESS: {
            auto return_status {BT::NodeStatus::FAILURE};

            auto resp {request_future_->get()};

            // if (resp->similarity > getInput<double>("similarity_threshold").value()) {
            //     RCLCPP_INFO_STREAM(node_->get_logger(), "Manage local storage responded with " << resp->similarity << ". Returning " << return_status << "!");
            //     return return_status;
            // }

            setOutput("load_files", std::make_shared<std::deque<std::string>>(resp->load_files.begin(), resp->load_files.end()));
            setOutput("unload_files", std::make_shared<std::deque<std::string>>(resp->unload_files.begin(), resp->unload_files.end()));
            return_status = BT::NodeStatus::SUCCESS;

            request_future_ = std::nullopt;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Manage local storage responded! Returning " << return_status << "! " << resp->load_files.size() << " files to load and " << resp->unload_files.size() << " files to unload.");
            return return_status;
        }
    }

    return BT::NodeStatus::FAILURE;
}

void ManageLocalStorage::onHalted()
{
    if (request_future_.has_value()) {
        service_client_->remove_pending_request(request_future_.value());
    }
}

} // namespace hyla_slam_behaviors
