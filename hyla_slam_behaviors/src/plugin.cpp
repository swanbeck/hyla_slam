#include <behaviortree_cpp/bt_factory.h>
#include "hyla_slam_behaviors/construct_pose.hpp"
#include "hyla_slam_behaviors/disable_localization.hpp"
#include "hyla_slam_behaviors/enable_localization.hpp"
#include "hyla_slam_behaviors/geodetic_to_ecef.hpp"
#include "hyla_slam_behaviors/get_localization_displacement.hpp"
#include "hyla_slam_behaviors/get_map_similarity.hpp"
#include "hyla_slam_behaviors/get_map.hpp"
#include "hyla_slam_behaviors/get_mapping_displacement.hpp"
#include "hyla_slam_behaviors/get_pose.hpp"
#include "hyla_slam_behaviors/index_data.hpp"
#include "hyla_slam_behaviors/load_data.hpp"
#include "hyla_slam_behaviors/lookup_hashes.hpp"
#include "hyla_slam_behaviors/manage_local_storage.hpp"
#include "hyla_slam_behaviors/publish_cloud.hpp"
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
    factory.registerNodeType<hyla_slam_behaviors::GetPose>("GetPose");
    factory.registerNodeType<hyla_slam_behaviors::IndexData>("IndexData");
    factory.registerNodeType<hyla_slam_behaviors::LoadData>("LoadData");
    factory.registerNodeType<hyla_slam_behaviors::LookupHashes>("LookupHashes");
    factory.registerNodeType<hyla_slam_behaviors::ManageLocalStorage>("ManageLocalStorage");
    factory.registerNodeType<hyla_slam_behaviors::PublishCloud>("PublishCloud");
    factory.registerNodeType<hyla_slam_behaviors::SetLocalizationEstimate>("SetLocalizationEstimate");
    factory.registerNodeType<hyla_slam_behaviors::SpawnRobotMarker>("SpawnRobotMarker");
    factory.registerNodeType<hyla_slam_behaviors::UnloadData>("UnloadData");
    factory.registerNodeType<hyla_slam_behaviors::UpdateLocalizationMap>("UpdateLocalizationMap");

    factory.addMetadataToManifest("ConstructPose", {
        {"description", "Constructs a PoseStamped message from position and orientation components."},
    });
    factory.addMetadataToManifest("DisableLocalization", {
        {"description", "Disables localization on the hyla_slam node."},
    });
    factory.addMetadataToManifest("EnableLocalization", {
        {"description", "Enables localization on the hyla_slam node."},
    });
    factory.addMetadataToManifest("GeodeticToEcef", {
        {"description", "Converts geodetic coordinates (latitude, longitude, height) to an ECEF PoseStamped."},
    });
    factory.addMetadataToManifest("GetLocalizationDisplacement", {
        {"description", "Gets the localization displacement and returns SUCCESS if it exceeds a linear or angular target threshold."},
    });
    factory.addMetadataToManifest("GetMap", {
        {"description", "Gets the current point cloud map from the hyla_slam node."},
    });
    factory.addMetadataToManifest("GetMapSimilarity", {
        {"description", "Gets the map similarity score and returns SUCCESS if it exceeds a provided threshold."},
    });
    factory.addMetadataToManifest("GetMappingDisplacement", {
        {"description", "Gets the mapping displacement and returns SUCCESS if it exceeds a linear or angular target threshold."},
    });
    factory.addMetadataToManifest("GetPose", {
        {"description", "Gets the current pose estimate from the hyla_slam node."},
    });
    factory.addMetadataToManifest("IndexData", {
        {"description", "Indexes a point cloud into the hyla_slam data store."},
    });
    factory.addMetadataToManifest("LoadData", {
        {"description", "Loads data into the hyla_slam node's active memory."},
    });
    factory.addMetadataToManifest("LookupHashes", {
        {"description", "Looks up data hashes for a given location index."},
    });
    factory.addMetadataToManifest("ManageLocalStorage", {
        {"description", "Determines which data files to load and unload based on current pose and map similarity."},
    });
    factory.addMetadataToManifest("PublishCloud", {
        {"description", "Publishes a point cloud to a specified ROS2 topic."},
    });
    factory.addMetadataToManifest("SetLocalizationEstimate", {
        {"description", "Sets the initial localization estimate pose on the hyla_slam node."},
    });
    factory.addMetadataToManifest("SpawnRobotMarker", {
        {"description", "Publishes a visualization marker for the robot coincident with a given TF frame."},
    });
    factory.addMetadataToManifest("UnloadData", {
        {"description", "Unloads data from the hyla_slam node's active memory."},
    });
    factory.addMetadataToManifest("UpdateLocalizationMap", {
        {"description", "Triggers an update of the localization map on the hyla_slam node."},
    });
}
