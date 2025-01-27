#include "hyla_slam_behaviors/geodetic_to_ecef.hpp"

constexpr double R_EARTH = 6371000.0; // Earth's radius in meters (assuming a perfect sphere)

// Convert degrees to radians
double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// Compute the 6DOF pose in ECEF
void computeECEFPose(double latitude, double longitude, double height, double extraAngleDeg, Eigen::Vector3d &position, Eigen::Matrix3d &rotationMatrix) {
    // Convert latitude, longitude, and extra angle to radians
    double lat = degToRad(latitude);
    double lon = degToRad(longitude);
    double extraAngle = degToRad(extraAngleDeg);

    // Step 1: Compute ECEF position
    double X = (R_EARTH + height) * std::cos(lat) * std::cos(lon);
    double Y = (R_EARTH + height) * std::cos(lat) * std::sin(lon);
    double Z = (R_EARTH + height) * std::sin(lat);
    position = Eigen::Vector3d(X, Y, Z);

    // Step 2: Compute the "up" vector (normalized position vector)
    Eigen::Vector3d upVector = position.normalized();

    // Step 3: Compute the "east" vector
    Eigen::Vector3d eastVector(-std::sin(lon), std::cos(lon), 0.0);
    eastVector.normalize();

    // Step 4: Compute the "north" vector (cross product of up and east)
    Eigen::Vector3d northVector = upVector.cross(eastVector);
    northVector.normalize();

    // Step 5: Apply extra rotation around the "up" vector
    Eigen::Matrix3d extraRotation;
    double c = std::cos(extraAngle);
    double s = std::sin(extraAngle);
    double u_x = upVector.x();
    double u_y = upVector.y();
    double u_z = upVector.z();

    // Rodrigues' rotation formula for a rotation matrix
    extraRotation << 
        c + u_x * u_x * (1 - c),      u_x * u_y * (1 - c) - u_z * s,  u_x * u_z * (1 - c) + u_y * s,
        u_y * u_x * (1 - c) + u_z * s, c + u_y * u_y * (1 - c),      u_y * u_z * (1 - c) - u_x * s,
        u_z * u_x * (1 - c) - u_y * s, u_z * u_y * (1 - c) + u_x * s, c + u_z * u_z * (1 - c);

    // Rotate the east and north vectors
    Eigen::Vector3d rotatedEast = extraRotation * eastVector;
    Eigen::Vector3d rotatedNorth = extraRotation * northVector;

    // Step 6: Construct the rotation matrix
    rotationMatrix.col(0) = rotatedEast;  // X-axis
    rotationMatrix.col(1) = rotatedNorth; // Y-axis
    rotationMatrix.col(2) = upVector;    // Z-axis
}

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
        BT::InputPort<double>("heading"),
        BT::InputPort<std::string>("frame"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),
    };
}

BT::NodeStatus GeodeticToEcef::tick()
{
    auto latitude {getInput<double>("latitude").value_or(0.0)};
    auto longitude {getInput<double>("longitude").value_or(0.0)};
    auto height {getInput<double>("height").value_or(0.0)};
    auto heading {getInput<double>("heading").value_or(0.0)};
    auto frame {getInput<std::string>("frame").value_or("world")};

    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;

    computeECEFPose(latitude, longitude, height, heading, pos, rot);

    Eigen::Quaterniond q(rot);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();
    pose.pose.position.z = pos.z();
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();

    setOutput<geometry_msgs::msg::PoseStamped>("pose", pose);

    std::cout << "(x, y, z) = (" << std::scientific << pos.x() << ", " << std::scientific << pos.y() << ", " << std::scientific << pos.z() << ")" << std::endl;
    std::cout << "(w, x, y, z) = (" << std::scientific << q.w() << ", " << std::scientific << q.x() << ", " << std::scientific << q.y() << ", " << std::scientific << q.z() << ")" << std::endl;

    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
