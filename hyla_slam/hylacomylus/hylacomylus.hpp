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
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <boost/multiprecision/cpp_int.hpp>

#include "surface_repair_common/point_type.h"
#include "chunk.hpp"
#include "hasher.hpp"
#include "types.hpp"

using namespace boost::multiprecision;

namespace hylacomylus {

class Hylacomylus {

public:
    Hylacomylus(const MappingConfig &config);
    ~Hylacomylus();

    void update(PointCloud::Ptr &cloud, const Sophus::SE3d &pose);
    PointCloud::Ptr sparseMap(const Sophus::SE3d &pose, const std::optional<double> &radius=std::nullopt, const std::optional<Sophus::SE3d> &projected_pose=std::nullopt);
    PointCloud::Ptr denseMap(const Sophus::SE3d &pose, const std::optional<double> &radius=std::nullopt, const std::optional<Sophus::SE3d> &projected_pose=std::nullopt);
    void unloadData();

    std::tuple<double, std::vector<std::string>, std::vector<std::string>> manageDisk(const Sophus::SE3d &pose, const double &threshold, const double &radius, const std::optional<Sophus::SE3d> &projected_pose=std::nullopt, const std::optional<std::set<uint256_t>> &search_hash_set=std::nullopt);

    std::set<uint256_t> lookupDiskHashes();
    std::set<uint256_t> lookupMemoryHashes();

private:
    std::tuple<double, std::set<uint256_t>, std::set<uint256_t>> jaccardSimilarity(const std::set<uint256_t> &s1, const std::set<uint256_t> &s2);
    std::set<uint256_t> searchLocalHashes(const Sophus::SE3d &pose, const double &radius, const std::set<uint256_t> &search_hashes, const std::optional<Sophus::SE3d> &projected_pose);
    std::set<uint256_t> computeLocalHashes(const Sophus::SE3d &pose, const double &radius, const std::optional<Sophus::SE3d> &projected_pose);
    void expandStorage(const std::set<uint256_t> &hashes);
    void pruneStorage(const std::set<uint256_t> &hashes);
    std::set<uint256_t> indexData(PointCloud::Ptr &cloud, const Sophus::SE3d &pose, const std::optional<std::uint32_t> &time_hash=std::nullopt);
    void saveScan(PointCloud::Ptr &cloud, const std::uint32_t &collection_id, const bool voxelize);
    std::set<uint256_t> updateHashMemory(std::set<uint256_t> &hashes);
    std::uint32_t generateTimeHash();

private:
    MappingConfig config_;
    Hasher hasher_;
    std::map<uint256_t, Chunk> dense_atlas_;
    std::map<uint256_t, Chunk> sparse_atlas_;
    std::deque<std::set<uint256_t>> hash_memory_;

    std::filesystem::path dense_chunk_path_;
    std::filesystem::path dense_scan_path_;
    std::filesystem::path sparse_chunk_path_;
    std::filesystem::path sparse_scan_path_;

}; // class Hylacomylus

} // namespace hylacomylus
