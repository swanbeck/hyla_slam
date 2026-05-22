#include "hyla_slam_behaviors/spawn_robot_marker.hpp"

namespace hyla_slam_behaviors {

SpawnRobotMarker::SpawnRobotMarker(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config), node_(rclcpp::Node::make_shared(name))
{
    pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "robot_marker", 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local()
    );
}

BT::PortsList SpawnRobotMarker::providedPorts()
{
    return {
        BT::InputPort<std::string>("frame_id", "TF frame ID the marker is attached to."),
        BT::InputPort<std::string>("mesh_path", "Path to a mesh resource for the marker. If not provided, a cube is used."),
    };
}

BT::NodeStatus SpawnRobotMarker::tick()
{
    auto frame_id {getInput<std::string>("frame_id").value()};
    auto mesh_path {getInput<std::string>("mesh_path")};

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "hyla_slam";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = geometry_msgs::msg::Pose();
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.frame_locked = true;

    if (mesh_path.has_value()) {
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = mesh_path.value();
        marker.mesh_use_embedded_materials = true;
    } else {
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.color.r = 0.8;
        marker.color.g = 0.33;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
    }

    pub_->publish(marker);

    RCLCPP_INFO_STREAM(node_->get_logger(), "Robot marker spawned coincident with frame " << frame_id << "!");
    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
