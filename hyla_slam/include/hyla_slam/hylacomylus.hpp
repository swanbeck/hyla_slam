#pragma once

#include <cstdint>
#include <chrono>
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <stack>
#include <filesystem>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include "surface_repair_common/point_type.h"
// #include "surface_repair_common/utils.hpp"

#include "chunk.hpp"
#include "hasher.hpp"
#include "utils.hpp"
#include "types.hpp"

using namespace std::chrono_literals;

namespace hylacomylus {

class Hylacomylus
{
public:
    Hylacomylus(const MappingConfig &config);

    ~Hylacomylus();

    void update(PointCloud::Ptr &cloud, const Pose3D &robot_pose);

    void dumpMemoryData();

    PointCloud::Ptr map();

private:
    std::uint32_t generateTimeHash();

    std::set<std::uint64_t> findLocalHashes(const Pose3D &robot_pose, const float &half_side_length);

    void rescopeStorage(const std::set<std::uint64_t> &hashes);

    void updateLocalMap(const Pose3D &robot_pose, const std::optional<std::set<std::uint64_t>> &additional_hashes=std::nullopt);
    
    std::set<std::uint64_t> indexData(PointCloud::Ptr &cloud, const Pose3D &robot_pose, const bool &add_data);

    void composeLocalMap(const std::set<std::uint64_t> &hashes);

    void deleteLastData();

    void deleteMappingResult(const MappingResult &result);

private:
    MappingConfig config_;
    std::unique_ptr<ChunkHasher> hasher_;
    std::unique_ptr<std::map<std::uint64_t, Chunk>> atlas_;
    std::unique_ptr<std::stack<MappingResult>> collection_history_;
    PointCloud::Ptr local_map_;

}; // class Hylacomylus

} // namespace hylacomylus
