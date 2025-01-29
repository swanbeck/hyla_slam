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
    PointCloud::Ptr denseMap();
    PointCloud::Ptr sparseMap();

    void saveRawScan(PointCloud::Ptr &cloud, const std::optional<std::uint32_t> &collection_id, const bool voxelize=false);

    std::uint32_t generateTimeHash();

private:

    std::set<uint256_t> findLocalHashes(const Sophus::SE3d &robot_pose, const float &half_side_length);

    void rescopeStorage(const std::set<uint256_t> &hashes, const std::optional<std::set<uint256_t>> &additional_hashes);
    
    std::set<uint256_t> indexData(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose, const bool &add_data, const std::optional<std::uint32_t> &time_hash=std::nullopt);

    void composeLocalMap(const std::set<uint256_t> &local_hashes, const std::optional<std::set<uint256_t>> &additional_hashes);

    // void deleteLastData();

    // void deleteMappingResult(const MappingResult &result);

    std::set<uint256_t> updateHashMemory(std::set<uint256_t> &hashes);

    void updateLocalMap(const Sophus::SE3d &robot_pose, const std::optional<std::set<uint256_t>> &additional_hashes=std::nullopt);

private:
    MappingConfig config_;
    ChunkHasher hasher_;
    std::unique_ptr<std::map<uint256_t, Chunk>> dense_atlas_;
    std::unique_ptr<std::map<uint256_t, Chunk>> sparse_atlas_;
    std::unique_ptr<std::stack<MappingResult>> collection_history_;
    PointCloud::Ptr dense_local_map_;
    PointCloud::Ptr sparse_local_map_;

    std::deque<std::set<uint256_t>> hash_memory_;
    std::set<uint256_t> most_recent_hash_set_;
    std::set<uint256_t> last_update_hash_set_;
    
    std::set<uint256_t> dense_hashes_;
    std::set<uint256_t> sparse_hashes_;

    std::filesystem::path chunk_path_;
    std::filesystem::path raw_path_;
    std::filesystem::path vox_chunk_path_;
    std::filesystem::path vox_scan_path_;

}; // class Hylacomylus

} // namespace hylacomylus
