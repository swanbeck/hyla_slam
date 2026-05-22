#include "hyla_slam_behaviors/publish_cloud.hpp"

namespace hyla_slam_behaviors {

PublishCloud::PublishCloud(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared(name);
}

BT::PortsList PublishCloud::providedPorts()
{
    return {
        BT::InputPort<std::string>("topic", "Topic name to publish the cloud to."),
        BT::InputPort<std::shared_ptr<sensor_msgs::msg::PointCloud2>>("cloud", "PointCloud2 message to publish."),
    };
}

BT::NodeStatus PublishCloud::tick()
{
    auto cloud_opt {getInput<std::shared_ptr<sensor_msgs::msg::PointCloud2>>("cloud")};
    if (!cloud_opt.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "No cloud provided!");
        return BT::NodeStatus::FAILURE;
    }
    auto topic_opt {getInput<std::string>("topic")};
    if (!topic_opt.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "No topic provided!");
        return BT::NodeStatus::FAILURE;
    }

    pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_opt.value_or("cloud"), 1);
    pub_->publish(*cloud_opt.value());
    RCLCPP_INFO_STREAM(node_->get_logger(), "Published cloud to " << topic_opt.value() << "...");

    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
