#include "hyla_slam_behaviors/construct_pose.hpp"

namespace hyla_slam_behaviors {

ConstructPose::ConstructPose(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList ConstructPose::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::Point>("point"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z"),
        BT::InputPort<double>("qw"),
        BT::InputPort<double>("qx"),
        BT::InputPort<double>("qy"),
        BT::InputPort<double>("qz"),
        BT::InputPort<std::string>("frame_id"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
}

BT::NodeStatus ConstructPose::tick()
{
    double x, y, z;
    auto point_opt {getInput<geometry_msgs::msg::Point>("point")};
    if (point_opt.has_value()) {
        x = point.value().x;
        y = point.value().y;
        z = point.value().z;
    } else {
        x = getInput<double>("x").value_or(0.0);
        y = getInput<double>("y").value_or(0.0);
        z = getInput<double>("z").value_or(0.0);
    }
    auto qw {getInput<double>("qw").value_or(1.0)};
    auto qx {getInput<double>("qx").value_or(0.0)};
    auto qy {getInput<double>("qy").value_or(0.0)};
    auto qz {getInput<double>("qz").value_or(0.0)};
    auto frame_id {getInput<std::string>("frame_id").value_or("world")};

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;

    setOutput<geometry_msgs::msg::PoseStamped>("pose", pose);

    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
