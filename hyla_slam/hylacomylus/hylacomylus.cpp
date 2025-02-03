#include "hylacomylus.hpp"

namespace hylacomylus {

Hylacomylus::Hylacomylus(const MappingConfig &config)
: config_(config), hasher_(config.chunk_discretization)
{
    if (!(std::filesystem::exists(config.data_dir))) {
        std::filesystem::create_directories(config.data_dir);
    }
    dense_chunk_path_ = std::filesystem::path(config.data_dir).append("dense_chunks");
    if (!(std::filesystem::exists(dense_chunk_path_))) {
        std::filesystem::create_directories(dense_chunk_path_);
    }
    dense_scan_path_ = std::filesystem::path(config.data_dir).append("dense_scans");
    if (!(std::filesystem::exists(dense_scan_path_))) {
        std::filesystem::create_directories(dense_scan_path_);
    }
    sparse_chunk_path_ = std::filesystem::path(config.data_dir).append("sparse_chunks");
    if (!(std::filesystem::exists(sparse_chunk_path_))) {
        std::filesystem::create_directories(sparse_chunk_path_);
    }
    sparse_scan_path_ = std::filesystem::path(config.data_dir).append("sparse_scans");
    if (!(std::filesystem::exists(sparse_scan_path_))) {
        std::filesystem::create_directories(sparse_scan_path_);
    }
}

Hylacomylus::~Hylacomylus()
{
    unloadData();
}

std::tuple<double, std::set<std::string>, std::set<std::string>, std::set<uint256_t>> Hylacomylus::manageDiskMemory(const Sophus::SE3d &pose, const double &threshold, const double &radius, const std::optional<Sophus::SE3d> &projected_pose, const std::optional<std::set<uint256_t>> &disk_hash_set)
{
    // get set of all chunks on disk (or have this saved? probably should be saved)
    std::set<uint256_t> disk_hashes;
    if (disk_hash_set.has_value()) {
        disk_hashes = disk_hash_set.value();
    } else {
        for (const auto &entry : std::filesystem::directory_iterator(dense_chunk_path_)) {
            if (entry.is_regular_file()) {
                std::string filename {entry.path().filename().string()};
                uint256_t hash {std::stoull(filename.substr(0, filename.find('.')))};
                disk_hashes.insert(hash);
            }
        }
    }

    // now get all hashes near the robot
    auto local_hashes {findLocalHashes(pose, radius, projected_pose)};

    // compute similarity
    auto jaccard_similarity = [](const std::set<uint256_t> &s1, const std::set<uint256_t> &s2) -> std::tuple<double, std::set<uint256_t>, std::set<uint256_t>> {
        std::set<uint256_t> intersection_set;
        std::set<uint256_t> union_set;

        std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(intersection_set, intersection_set.begin()));
        std::set_union(s1.begin(), s1.end(), s2.begin(), s2.end(), std::inserter(union_set, union_set.begin()));

        double similarity = union_set.empty() ? 1.0 : static_cast<double>(intersection_set.size()) / union_set.size();

        return std::make_tuple(similarity, intersection_set, union_set);
    };

    // compute the set of items in disk_hashes not in local_hashes and vice versa
    std::set<uint256_t> disk_not_local;
    std::set<uint256_t> local_not_disk;

    auto [similarity, intersection_set, union_set] = jaccard_similarity(disk_hashes, local_hashes);

    // don't bother computing expensive set difference if not sufficiently dissimilar
    if (similarity > threshold) {
        std::set<std::string> disk_not_local_paths;
        std::set<std::string> local_not_disk_paths;
        return std::make_tuple(similarity, disk_not_local_paths, local_not_disk_paths, disk_hashes);
    }

    std::set_difference(disk_hashes.begin(), disk_hashes.end(), local_hashes.begin(), local_hashes.end(), std::inserter(disk_not_local, disk_not_local.begin()));
    std::set_difference(local_hashes.begin(), local_hashes.end(), disk_hashes.begin(), disk_hashes.end(), std::inserter(local_not_disk, local_not_disk.begin()));

    // convert hashes to paths
    std::set<std::string> disk_not_local_paths;
    std::set<std::string> local_not_disk_paths;

    for (const auto &hash : disk_not_local) {
        std::ostringstream oss;
        oss << hash;
        disk_not_local_paths.insert((dense_chunk_path_ / (oss.str() + ".pcd")).string());
    }

    for (const auto &hash : local_not_disk) {
        std::ostringstream oss;
        oss << hash;
        local_not_disk_paths.insert((dense_chunk_path_ / (oss.str() + ".pcd")).string());
    }

    // remove disk not local from storage if it exists so it is complete and ready to be offloaded
    pruneStorage(disk_not_local);

    return std::make_tuple(similarity, disk_not_local_paths, local_not_disk_paths, disk_hashes);
}

void Hylacomylus::update(PointCloud::Ptr &cloud, const Sophus::SE3d &pose)
{
    auto scan_hashes {indexData(cloud, pose)};
    auto recent_hashes {updateHashMemory(scan_hashes)};
    pruneStorage(recent_hashes);
    expandStorage(recent_hashes);
}

PointCloud::Ptr Hylacomylus::sparseMap(const Sophus::SE3d &pose, const std::optional<double> &radius, const std::optional<Sophus::SE3d> &projected_pose) {
    const auto hashes {findLocalHashes(pose, radius.has_value() ? radius.value() : config_.sparse_map_radius, projected_pose)};
    expandStorage(hashes);

    PointCloud::Ptr map (new PointCloud);
    for (const auto &hash : hashes) {
        *map += *(sparse_atlas_.at(hash).chunk);
    }

    return map;
}

PointCloud::Ptr Hylacomylus::denseMap(const Sophus::SE3d &pose, const std::optional<double> &radius, const std::optional<Sophus::SE3d> &projected_pose) {
    const auto hashes {findLocalHashes(pose, radius.has_value() ? radius.value() : config_.dense_map_radius, projected_pose)};
    expandStorage(hashes);
    
    PointCloud::Ptr map (new PointCloud);
    for (const auto &hash : hashes) {
        *map += *(dense_atlas_.at(hash).chunk);
    }

    return map;
}

void Hylacomylus::unloadData() {
    for (auto &entry : dense_atlas_) {
        entry.second.unload();
    }

    for (auto &entry : sparse_atlas_) {
        entry.second.unload();
    }
}

std::set<uint256_t> Hylacomylus::findLocalHashes(const Sophus::SE3d &pose, const double &radius, const std::optional<Sophus::SE3d> &projected_pose) {
    std::set<uint256_t> hashes;
    double increment {static_cast<double>(config_.chunk_discretization) / 2};

    const Eigen::Vector3d foci1 {pose.translation()};
    const Eigen::Vector3d foci2 {projected_pose.has_value() ? pose.translation() + 2 * (projected_pose.value().translation() - pose.translation()) : foci1};

    const Eigen::Vector3d midpoint {(foci1 + foci2) / 2};
    const double a {std::max(radius, (foci1 - foci2).norm() / 2)};

    for (double i = (midpoint.x() - a); i < (midpoint.x() + a); i += increment) {
        for (double j = (midpoint.y() - a); j < (midpoint.y() + a); j += increment) {
            for (double k = (midpoint.z() - a); k < (midpoint.z() + a); k += increment) {
                Eigen::Vector3d point(i, j, k);
                double distance1 {(point - foci1).norm()};
                double distance2 {(point - foci2).norm()};
                if (distance1 + distance2 <= 2 * a) {
                    hashes.insert(hasher_.generateHash(point));
                }
            }
        }
    }

    return hashes;
}

void Hylacomylus::expandStorage(const std::set<uint256_t> &hashes) {
    for (const auto &hash : hashes) {
        if (config_.maintain_dense_chunks) {
            if (!dense_atlas_.contains(hash)) {
                dense_atlas_.insert({hash, Chunk(hash, dense_chunk_path_.string())});
            }
            dense_atlas_.at(hash).load();
        }

        if (config_.maintain_sparse_chunks) {
            if (!sparse_atlas_.contains(hash)) {
                sparse_atlas_.insert({hash, Chunk(hash, sparse_chunk_path_.string(), true, config_.sparse_voxel_size)});
            }
            sparse_atlas_.at(hash).load();
        }
    }
}

void Hylacomylus::pruneStorage(const std::set<uint256_t> &hashes) {
    if (config_.maintain_dense_chunks) {
        std::vector<uint256_t> dense_erase_keys;
        for (auto &entry : dense_atlas_) {
            if (!(hashes.contains(entry.first))) {
                entry.second.unload();
                dense_erase_keys.push_back(entry.first);
            }
        }
        for (const auto &key : dense_erase_keys) {
            dense_atlas_.erase(key);
        }
    }

    if (config_.maintain_sparse_chunks) {
        std::vector<uint256_t> sparse_erase_keys;
        for (auto &entry : sparse_atlas_) {
            if (!(hashes.contains(entry.first))) {
                entry.second.unload();
                sparse_erase_keys.push_back(entry.first);
            }
        }
        for (const auto &key : sparse_erase_keys) {
            sparse_atlas_.erase(key);
        }
    }
}

std::set<uint256_t> Hylacomylus::indexData(PointCloud::Ptr &cloud, const Sophus::SE3d &pose, const std::optional<std::uint32_t> &time_hash) {
    std::uint32_t collection_id {time_hash.value_or(Hylacomylus::generateTimeHash())};

    if (config_.save_dense_scans) {
        Hylacomylus::saveScan(cloud, collection_id, false);
    }
    if (config_.save_sparse_scans) {
        Hylacomylus::saveScan(cloud, collection_id, true);
    }

    PointCloud::Ptr transformed_cloud (new PointCloud);
    pcl::transformPointCloudWithNormals(*cloud, *transformed_cloud, pose.matrix());

    std::set<uint256_t> scan_hashes;
    for (std::size_t i = 0; i < transformed_cloud->points.size(); i++) {
        uint256_t hash {hasher_.generateHash(transformed_cloud->points[i])};
        if (!(scan_hashes.contains(hash))) {
            scan_hashes.insert(hash);
        }

        transformed_cloud->points[i].collection_id = collection_id;
        transformed_cloud->points[i].sensor_a = pose.translation().x();
        transformed_cloud->points[i].sensor_b = pose.translation().y();
        transformed_cloud->points[i].sensor_c = pose.translation().z();

        if (config_.maintain_dense_chunks) {
            if (!(dense_atlas_.contains(hash))) {
                dense_atlas_.insert({hash, Chunk(hash, dense_chunk_path_.string())});
                dense_atlas_.at(hash).load();
            }

            if ((dense_atlas_.at(hash).chunk->points.size() >= config_.max_points_per_dense_chunk) && (config_.max_points_per_dense_chunk > 0)) {
                continue;
            }

            dense_atlas_.at(hash).chunk->points.push_back(transformed_cloud->points[i]);
        }

        if (config_.maintain_sparse_chunks) {
            if (!(sparse_atlas_.contains(hash))) {
                sparse_atlas_.insert({hash, Chunk(hash, sparse_chunk_path_.string(), true, config_.sparse_voxel_size)});
                sparse_atlas_.at(hash).load();
            }

            if ((sparse_atlas_.at(hash).chunk->points.size() >= config_.max_points_per_sparse_chunk) && (config_.max_points_per_sparse_chunk > 0)) {
                continue;
            }

            sparse_atlas_.at(hash).chunk->points.push_back(transformed_cloud->points[i]);
        }
    }

    return scan_hashes;
}

void Hylacomylus::saveScan(PointCloud::Ptr &cloud, const std::uint32_t &collection_id, const bool voxelize) {
    std::filesystem::path save_path = voxelize ? sparse_scan_path_ / (std::to_string(collection_id) + ".pcd") : dense_scan_path_ / (std::to_string(collection_id) + ".pcd");

    if (std::filesystem::exists(save_path)) {
        return; 
    }
    
    if (voxelize) {
        pcl::VoxelGrid<Point> grid;
        grid.setInputCloud(cloud);
        grid.setLeafSize(config_.sparse_voxel_size, config_.sparse_voxel_size, config_.sparse_voxel_size);
        PointCloud::Ptr voxelized_cloud (new PointCloud);
        grid.filter(*voxelized_cloud);
        pcl::io::savePCDFileBinary(save_path.string(), *voxelized_cloud);
    } else {
        cloud->height = 1;
        cloud->width = cloud->points.size();
        pcl::io::savePCDFileBinary(save_path.string(), *cloud);
    }
}

std::set<uint256_t> Hylacomylus::updateHashMemory(std::set<uint256_t> &hashes) {
    if (config_.scan_memory_horizon == 0) { return hashes; }

    hash_memory_.push_back(hashes);
    while (hash_memory_.size() > config_.scan_memory_horizon) {
        hash_memory_.pop_front();
    }

    std::set<uint256_t> recent_hashes;
    for (const auto &hash_set : hash_memory_) {
        recent_hashes.insert(hash_set.begin(), hash_set.end());
    }

    return recent_hashes;
}

std::uint32_t Hylacomylus::generateTimeHash() {
    auto currentTime = std::chrono::system_clock::now();
    auto durationSinceEpoch = currentTime.time_since_epoch();
    return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch).count() % UINT32_MAX);
}

} // namespace hylacomylus
