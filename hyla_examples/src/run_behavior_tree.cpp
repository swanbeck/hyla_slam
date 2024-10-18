#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <hyla_slam_behaviors/get_map.hpp>
#include <hyla_slam_behaviors/index_data.hpp>
#include <hyla_slam_behaviors/unload_data.hpp>
#include <hyla_slam_behaviors/enable_localization.hpp>
#include <hyla_slam_behaviors/disable_localization.hpp>
#include <hyla_slam_behaviors/update_localization_map.hpp>
#include <hyla_slam_behaviors/get_mapping_displacement.hpp>
#include <hyla_slam_behaviors/set_localization_estimate.hpp>
#include <hyla_slam_behaviors/get_localization_displacement.hpp>

#include <watertender_behaviors/get_point_cloud_data.hpp>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("hyla_examples_behavior_tree");

    const std::string source_package = nh->declare_parameter<std::string>("source_package");
    const std::string behavior_tree_file = nh->declare_parameter<std::string>("behavior_tree_file");

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<hyla_slam_behaviors::GetMap>("GetMap");
    factory.registerNodeType<hyla_slam_behaviors::IndexData>("IndexData");
    factory.registerNodeType<hyla_slam_behaviors::UnloadData>("UnloadData");
    factory.registerNodeType<hyla_slam_behaviors::EnableLocalization>("EnableLocalization");
    factory.registerNodeType<hyla_slam_behaviors::DisableLocalization>("DisableLocalization");
    factory.registerNodeType<hyla_slam_behaviors::UpdateLocalizationMap>("UpdateLocalizationMap");
    factory.registerNodeType<hyla_slam_behaviors::GetMappingDisplacement>("GetMappingDisplacement");
    factory.registerNodeType<hyla_slam_behaviors::SetLocalizationEstimate>("SetLocalizationEstimate");
    factory.registerNodeType<hyla_slam_behaviors::GetLocalizationDisplacement>("GetLocalizationDisplacement");
    factory.registerNodeType<watertender_behaviors::GetPointCloudData>("GetPointCloudData");

    const std::string package_share_dir = ament_index_cpp::get_package_share_directory(source_package);
    auto tree = factory.createTreeFromFile(std::filesystem::path(package_share_dir).append(behavior_tree_file));

    // Wait for tf data in watertender to be populated
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    tree.tickWhileRunning();

    RCLCPP_INFO(nh->get_logger(), "Behavior Tree execution finished");
    return 0;
}