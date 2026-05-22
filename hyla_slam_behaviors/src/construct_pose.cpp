#include "hyla_slam_behaviors/construct_pose.hpp"

namespace hyla_slam_behaviors {

ConstructPose::ConstructPose(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList ConstructPose::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<geometry_msgs::msg::Point>>("point", "Optional Point message; if provided, x/y/z values are taken from it instead of the scalar ports."),
        BT::InputPort<double>("x", 0.0, "X position component. Used only if point is not provided."),
        BT::InputPort<double>("y", 0.0, "Y position component. Used only if point is not provided."),
        BT::InputPort<double>("z", 0.0, "Z position component. Used only if point is not provided."),
        BT::InputPort<double>("qw", 1.0, "Quaternion w component."),
        BT::InputPort<double>("qx", 0.0, "Quaternion x component."),
        BT::InputPort<double>("qy", 0.0, "Quaternion y component."),
        BT::InputPort<double>("qz", 0.0, "Quaternion z component."),
        BT::InputPort<std::string>("frame_id", "world", "TF frame ID for the pose header."),
        BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", "Constructed PoseStamped message.")
    };
}

BT::NodeStatus ConstructPose::tick()
{
    double x, y, z;
    auto point_opt {getInput<std::shared_ptr<geometry_msgs::msg::Point>>("point")};
    if (point_opt.has_value()) {
        x = point_opt.value()->x;
        y = point_opt.value()->y;
        z = point_opt.value()->z;
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

    setOutput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", std::make_shared<geometry_msgs::msg::PoseStamped>(pose));

    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
