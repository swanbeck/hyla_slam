#include <behaviortree_cpp/bt_factory.h>
#include "hyla_slam_behaviors/construct_pose.hpp"
#include "hyla_slam_behaviors/disable_localization.hpp"
#include "hyla_slam_behaviors/enable_localization.hpp"
#include "hyla_slam_behaviors/geodetic_to_ecef.hpp"
#include "hyla_slam_behaviors/get_localization_displacement.hpp"
#include "hyla_slam_behaviors/get_map_similarity.hpp"
#include "hyla_slam_behaviors/get_map.hpp"
#include "hyla_slam_behaviors/get_mapping_displacement.hpp"
#include "hyla_slam_behaviors/index_data.hpp"
#include "hyla_slam_behaviors/manage_local_storage.hpp"
#include "hyla_slam_behaviors/set_localization_estimate.hpp"
#include "hyla_slam_behaviors/spawn_robot_marker.hpp"
#include "hyla_slam_behaviors/unload_data.hpp"
#include "hyla_slam_behaviors/update_localization_map.hpp"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory) {
    factory.registerNodeType<hyla_slam_behaviors::ConstructPose>("ConstructPose");
    factory.registerNodeType<hyla_slam_behaviors::DisableLocalization>("DisableLocalization");
    factory.registerNodeType<hyla_slam_behaviors::EnableLocalization>("EnableLocalization");
    factory.registerNodeType<hyla_slam_behaviors::GeodeticToEcef>("GeodeticToEcef");
    factory.registerNodeType<hyla_slam_behaviors::GetLocalizationDisplacement>("GetLocalizationDisplacement");
    factory.registerNodeType<hyla_slam_behaviors::GetMap>("GetMap");
    factory.registerNodeType<hyla_slam_behaviors::GetMapSimilarity>("GetMapSimilarity");
    factory.registerNodeType<hyla_slam_behaviors::GetMappingDisplacement>("GetMappingDisplacement");
    factory.registerNodeType<hyla_slam_behaviors::IndexData>("IndexData");
    factory.registerNodeType<hyla_slam_behaviors::ManageLocalStorage>("ManageLocalStorage");
    factory.registerNodeType<hyla_slam_behaviors::SetLocalizationEstimate>("SetLocalizationEstimate");
    factory.registerNodeType<hyla_slam_behaviors::SpawnRobotMarker>("SpawnRobotMarker");
    factory.registerNodeType<hyla_slam_behaviors::UnloadData>("UnloadData");
    factory.registerNodeType<hyla_slam_behaviors::UpdateLocalizationMap>("UpdateLocalizationMap");
}
