#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "hyla_kiss/hyla_kiss.hpp"
#include "hyla_kiss/kiss_icp_utils.hpp"

#include "hyla_sam/hyla_sam.hpp"

#include "hylacomylus/hylacomylus.hpp"
#include "hylacomylus/types.hpp"
#include "hylacomylus/utils.hpp"

#include "hyla_slam_parameters.hpp"

#include <std_srvs/srv/trigger.hpp>
#include "hyla_slam_interfaces/srv/get_displacement.hpp"
#include "hyla_slam_interfaces/srv/get_map.hpp"
#include "hyla_slam_interfaces/srv/get_map_similarity.hpp"
#include "hyla_slam_interfaces/srv/index_data.hpp"
#include "hyla_slam_interfaces/srv/set_pose.hpp"

namespace hyla_slam {

struct SamNode {
    Sophus::SE3d odom_pose;
    Sophus::SE3d rel_odom_pose;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scan;

    SamNode(const Sophus::SE3d odom, const Sophus::SE3d rel_odom, const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    : odom_pose(odom), rel_odom_pose(rel_odom), scan(msg)
    {}
}; // struct Node

class PoseGraph : public rclcpp::Node
{
public:
    PoseGraph(const rclcpp::NodeOptions &opts);

private:
    void receiveScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_msg);

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> sub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<hyla_slam::ParamListener> param_listener_;
    hyla_slam::Params params_;

    std::unique_ptr<hyla_kiss::HylaKiss> localizer_;
    std::unique_ptr<hyla_sam::HylaSam> sam_;

    hyla_kiss::KissConfig localization_config_;

    double dist_since_update_ {};
    std::optional<Sophus::SE3d> last_pose_;
    
    std::vector<SamNode> nodes_;

}; // class PoseGraph

} // namespace hyla_slam
