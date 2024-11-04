#pragma once

#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/action_node.h>

namespace hyla_slam_behaviors {

class ConstructPose : public BT::SyncActionNode
{
public:
    ConstructPose(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:

}; // class ConstructPose

} // namespace hyla_slam_behaviors
