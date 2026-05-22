#include "hyla_slam_behaviors/geodetic_to_ecef.hpp"

constexpr double R_EARTH = 6371000.0; // Earth's radius in meters (assuming a perfect sphere)

double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

void computeECEFPose(double latitude, double longitude, double height, double extraAngleDeg, Eigen::Vector3d &position, Eigen::Matrix3d &rotationMatrix) {
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
    rotationMatrix.col(0) = rotatedEast;
    rotationMatrix.col(1) = rotatedNorth;
    rotationMatrix.col(2) = upVector;
}

namespace hyla_slam_behaviors {

GeodeticToEcef::GeodeticToEcef(const std::string name, const BT::NodeConfig &config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList GeodeticToEcef::providedPorts()
{
    return {
        BT::InputPort<double>("latitude", "Geodetic latitude in degrees."),
        BT::InputPort<double>("longitude", "Geodetic longitude in degrees."),
        BT::InputPort<double>("height", "Height above the Earth's surface in meters."),
        BT::InputPort<double>("heading", "Extra rotation angle around the up vector in degrees."),
        BT::InputPort<std::string>("frame", "TF frame ID for the output pose header."),
        BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", "Computed ECEF PoseStamped."),
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

    setOutput<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose", std::make_shared<geometry_msgs::msg::PoseStamped>(pose));

    return BT::NodeStatus::SUCCESS;
}

} // namespace hyla_slam_behaviors
