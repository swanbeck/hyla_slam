#include "hyla_slam_behaviors/geodetic_to_ecef.hpp"

namespace hyla_slam_behaviors {

GeodeticToEcef::GeodeticToEcef(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList GeodeticToEcef::providedPorts()
{
    return {
        BT::InputPort<double>("latitude"),
        BT::InputPort<double>("longitude"),
        BT::InputPort<double>("height"),
        BT::OutputPort<geometry_msgs::msg::Point>("point"),
    };
}

BT::NodeStatus GeodeticToEcef::tick()
{
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates

    auto phi {getInput<double>("latitude").value_or(0.0)};
    auto lambda {getInput<double>("longitude").value_or(0.0)};
    auto h {getInput<double>("height").value_or(0.0)};

    // equatorial radius is 6,378.1370 km
    // polar radius is 6,356.7523 km
    double a_ {6.3781370e6};
    double b_ {6.3567523e6};

    double e_sqrd {1 - (pow(b_, 2) / pow(a_, 2))};

    double N {a_ / (sqrt((1 - e_sqrd) * pow(sin(phi), 2)))};

    double x {(N + h) * cos(phi) * cos(lambda)};
    double y {(N + h) * cos(phi) * sin(lambda)};
    double z {((1 - e_sqrd) * N + h) * sin(phi)};

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    setOutput<geometry_msgs::msg::PointStamped>("point", point);

    std::cout << "(x, y, z) = (" << std::scientific << x << ", " << std::scientific << y << ", " << std::scientific << z << ")" << std::endl;

    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
