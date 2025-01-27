#pragma once

#include <memory>
#include <iostream>
#include <iomanip>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <behaviortree_cpp/action_node.h>

namespace hyla_slam_behaviors {

class GeodeticToEcef : public BT::SyncActionNode
{
public:
    GeodeticToEcef(const std::string name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:



}; // class GeodeticToEcef

} // namespace hyla_slam_behaviors
