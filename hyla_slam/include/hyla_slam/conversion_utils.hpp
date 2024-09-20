#pragma once

#include <vector>
#include <Eigen/Dense>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace conversion_utils {

Eigen::Matrix4d pose2TransformationMatrix(const geometry_msgs::msg::Pose &pose)
{
    Eigen::Quaterniond q;
    q.w() = pose.orientation.w;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            T(i, j) = R(i, j);
        }
    }

    T(0,3) = pose.position.x;
    T(1,3) = pose.position.y;
    T(2,3) = pose.position.z;

    return T;
}

geometry_msgs::msg::Transform transformationMatrix2Transform(const Eigen::Matrix4d &matrix)
{
    geometry_msgs::msg::Transform transform;
    
    Eigen::Quaterniond q((Eigen::Matrix3d)matrix.block(0, 0, 3, 3));
    
    transform.rotation.w = q.w();
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.translation.x = matrix(0, 3);
    transform.translation.y = matrix(1, 3);
    transform.translation.z = matrix(2, 3);
    
    return transform;
}

} // namespace conversion_utils
