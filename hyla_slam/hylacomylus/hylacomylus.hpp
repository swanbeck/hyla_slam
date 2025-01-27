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
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <boost/multiprecision/cpp_int.hpp>
using namespace boost::multiprecision;

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

    std::set<uint256_t> update(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose);

    void dumpMemoryData();

    PointCloud::Ptr map();

private:
    std::uint32_t generateTimeHash();

    std::set<uint256_t> findLocalHashes(const Sophus::SE3d &robot_pose, const float &half_side_length);

    void rescopeStorage(const std::set<uint256_t> &hashes);
    
    std::set<uint256_t> indexData(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose, const bool &add_data);

    void composeLocalMap(const std::set<uint256_t> &hashes);

    void deleteLastData();

    void deleteMappingResult(const MappingResult &result);

    std::set<uint256_t> updateHashMemory(std::set<uint256_t> &hashes);

    void updateLocalMap(const Sophus::SE3d &robot_pose, const std::optional<std::set<uint256_t>> &additional_hashes=std::nullopt);

private:
    MappingConfig config_;
    std::unique_ptr<ChunkHasher> hasher_;
    std::unique_ptr<std::map<uint256_t, Chunk>> atlas_;
    std::unique_ptr<std::stack<MappingResult>> collection_history_;
    PointCloud::Ptr local_map_;

    std::deque<std::set<uint256_t>> hash_memory_;
    std::set<uint256_t> most_recent_hash_set_;
    std::set<uint256_t> last_update_hash_set_;

}; // class Hylacomylus

} // namespace hylacomylus
