#include "hylacomylus.hpp"

namespace hylacomylus {

Hylacomylus::Hylacomylus(const MappingConfig &config)
: config_(config), hasher_(config.chunk_discretization), dense_local_map_(new PointCloud), sparse_local_map_(new PointCloud)
{
    dense_atlas_ = std::make_unique<std::map<uint256_t, Chunk>>();
    sparse_atlas_ = std::make_unique<std::map<uint256_t, Chunk>>();
    collection_history_ = std::make_unique<std::stack<MappingResult>>();

    if (!(std::filesystem::exists(config.data_dir))) {
        std::filesystem::create_directories(config.data_dir);
    }

    chunk_path_ = std::filesystem::path(config.data_dir).append("chunks");
    if (!(std::filesystem::exists(chunk_path_))) {
        std::filesystem::create_directories(chunk_path_);
    }

    raw_path_ = std::filesystem::path(config.data_dir).append("scans");
    if (!(std::filesystem::exists(raw_path_))) {
        std::filesystem::create_directories(raw_path_);
    }

    vox_chunk_path_ = std::filesystem::path(config.data_dir).append("voxelized_chunks");
    if (!(std::filesystem::exists(vox_chunk_path_))) {
        std::filesystem::create_directories(vox_chunk_path_);
    }

    vox_scan_path_ = std::filesystem::path(config.data_dir).append("voxelized_scans");
    if (!(std::filesystem::exists(vox_scan_path_))) {
        std::filesystem::create_directories(vox_scan_path_);
    }

    updateLocalMap(Sophus::SE3d());
}

Hylacomylus::~Hylacomylus()
{
    dumpMemoryData();
}

std::uint32_t Hylacomylus::generateTimeHash()
{
    auto currentTime = std::chrono::system_clock::now();
    auto durationSinceEpoch = currentTime.time_since_epoch();
    return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch).count() % UINT32_MAX);
}

std::set<uint256_t> Hylacomylus::findLocalHashes(const Sophus::SE3d &robot_pose, const std::optional<Sophus::SE3d> &projected_pose)
{
    std::set<uint256_t> hashes;
    double increment {static_cast<double>(config_.chunk_discretization) / 2};

    Eigen::Vector3d foci1 = robot_pose.translation();
    Eigen::Vector3d foci2 = projected_pose.has_value() ? projected_pose.value().translation() + 3 * (projected_pose.value().translation() - robot_pose.translation()) : foci1;

    Eigen::Vector3d midpoint = (foci1 + foci2) / 2;
    const double a = std::max(config_.half_side_length, (foci1 - foci2).norm() / 2);

    std::cout << "Using " << a << "; " << (foci1 - foci2).norm() / 2 << " -> " << foci1.transpose() << "; " << foci2.transpose() << std::endl;

    for (double i = (midpoint.x() - a); i < (midpoint.x() + a); i += increment) {
        for (double j = (midpoint.y() - a); j < (midpoint.y() + a); j += increment) {
            for (double k = (midpoint.z() - a); k < (midpoint.z() + a); k += increment) {
                Eigen::Vector3d point(i, j, k);
                double distance1 = (point - foci1).norm();
                double distance2 = (point - foci2).norm();
                if (distance1 + distance2 <= 2 * a) {
                    hashes.insert(hasher_.generateHash(point));
                }
            }
        }
    }

    std::cout << "Local hashes: " << hashes.size() << std::endl;
    return hashes;
}

void Hylacomylus::rescopeStorage(const std::set<uint256_t> &hashes, const std::optional<std::set<uint256_t>> &additional_hashes)
{
    for (const auto &hash : hashes) {
        if (!(dense_atlas_->contains(hash))) {
            dense_atlas_->insert({hash, Chunk(hash, chunk_path_.string())});
        }
        dense_atlas_->at(hash).loadChunk();

        if (!(sparse_atlas_->contains(hash))) {
            sparse_atlas_->insert({hash, Chunk(hash, vox_chunk_path_.string(), true, config_.voxel_size)});
        }
        sparse_atlas_->at(hash).loadChunk();
    }

    std::vector<uint256_t> dense_erase_keys;
    for (auto &entry : *dense_atlas_) {
        if (!(hashes.contains(entry.first))) {
            entry.second.unloadChunk();
            dense_erase_keys.push_back(entry.first);
        }
    }

    for (const auto &key : dense_erase_keys) {
        dense_atlas_->erase(key);
    }

    if (additional_hashes.has_value()) {
        for (const auto &hash : additional_hashes.value()) {
            if (!(sparse_atlas_->contains(hash))) {
                sparse_atlas_->insert({hash, Chunk(hash, vox_chunk_path_.string(), true, config_.voxel_size)});
            }
            sparse_atlas_->at(hash).loadChunk();
        }
    }

    std::vector<uint256_t> sparse_erase_keys;
    for (auto &entry : *sparse_atlas_) {
        if (!(hashes.contains(entry.first)) && !(additional_hashes.has_value() && additional_hashes.value().contains(entry.first))) {
            entry.second.unloadChunk();
            sparse_erase_keys.push_back(entry.first);
        }
    }

    for (const auto &key : sparse_erase_keys) {
        sparse_atlas_->erase(key);
    }
}

std::set<uint256_t> Hylacomylus::update(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose, const std::optional<Sophus::SE3d> &projected_pose)
{
    auto data_hashes {indexData(cloud, robot_pose, config_.active_mapping)};
    auto recent_hashes {updateHashMemory(data_hashes)};
    updateLocalMap(robot_pose, config_.persist_recent_chunks ? std::optional(recent_hashes) : std::nullopt, projected_pose);
    return recent_hashes;
}

void Hylacomylus::updateLocalMap(const Sophus::SE3d &robot_pose, const std::optional<std::set<uint256_t>> &additional_hashes, const std::optional<Sophus::SE3d> &projected_pose)
{
    auto local_hashes {findLocalHashes(robot_pose, projected_pose)};
    rescopeStorage(local_hashes, additional_hashes);
    composeLocalMap(local_hashes, additional_hashes);
}

std::set<uint256_t> Hylacomylus::updateHashMemory(std::set<uint256_t> &hashes)
{
    if (config_.recent_scan_memory == 0) { return hashes; }

    hash_memory_.push_back(hashes);
    while (hash_memory_.size() > config_.recent_scan_memory) {
        hash_memory_.pop_front();
    }

    std::set<uint256_t> recent_hashes;
    for (const auto &hash_set : hash_memory_) {
        recent_hashes.insert(hash_set.begin(), hash_set.end());
    }

    return recent_hashes;
}

void Hylacomylus::saveRawScan(PointCloud::Ptr &cloud, const std::optional<std::uint32_t> &collection_id, const bool voxelize)
{
    std::uint32_t collection_hash = collection_id.value_or(Hylacomylus::generateTimeHash());

    std::filesystem::path save_path = voxelize ? vox_scan_path_ / (std::to_string(collection_hash) + ".pcd") : raw_path_ / (std::to_string(collection_hash) + ".pcd");

    if (utils::checkFileExistence(save_path)) {
        std::cout << "[WARN] file " << save_path << " already exists!" << std::endl;
        return; 
    }
    
    if (voxelize) {
        pcl::VoxelGrid<Point> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
        PointCloud::Ptr voxelized_cloud (new PointCloud);
        sor.filter(*voxelized_cloud);
        pcl::io::savePCDFileBinary(save_path.string(), *voxelized_cloud);
    } else {
        cloud->height = 1;
        cloud->width = cloud->points.size();
        pcl::io::savePCDFileBinary(save_path.string(), *cloud);
    }
}

std::set<uint256_t> Hylacomylus::indexData(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose, const bool &add_data, const std::optional<std::uint32_t> &time_hash)
{
    std::uint32_t collection_hash = time_hash.value_or(Hylacomylus::generateTimeHash());
    std::set<uint256_t> data_hashes;

    for (std::size_t i = 0; i < cloud->points.size(); i++) {
        uint256_t hash {hasher_.generateHash(cloud->points[i])};
        if (!(data_hashes.contains(hash))) {
            data_hashes.insert(hash);
        }

        if (add_data) {
            cloud->points[i].collection_id = collection_hash;
            cloud->points[i].sensor_a = robot_pose.translation().x();
            cloud->points[i].sensor_b = robot_pose.translation().y();
            cloud->points[i].sensor_c = robot_pose.translation().z();

            if (!(dense_atlas_->contains(hash))) {
                dense_atlas_->insert({hash, Chunk(hash, chunk_path_.string())});
                dense_atlas_->at(hash).loadChunk();
            }

            if (!(sparse_atlas_->contains(hash))) {
                sparse_atlas_->insert({hash, Chunk(hash, vox_chunk_path_.string(), true, config_.voxel_size)});
                sparse_atlas_->at(hash).loadChunk();
            }

            dense_atlas_->at(hash).chunk->points.push_back(cloud->points[i]);
            sparse_atlas_->at(hash).chunk->points.push_back(cloud->points[i]);
        }
    }

    if (add_data) {
        collection_history_->push(MappingResult(collection_hash, data_hashes));
    }

    return data_hashes;
}

void Hylacomylus::dumpMemoryData()
{
    for (auto &entry : *dense_atlas_) {
        entry.second.unloadChunk();
    }

    for (auto &entry : *sparse_atlas_) {
        entry.second.unloadChunk();
    }
}

void Hylacomylus::composeLocalMap(const std::set<uint256_t> &local_hashes, const std::optional<std::set<uint256_t>> &additional_hashes)
{
    dense_local_map_->clear();
    sparse_local_map_->clear();

    for (const auto &hash : local_hashes) {
        if (!(dense_atlas_->contains(hash))) {
            dense_atlas_->insert({hash, Chunk(hash, chunk_path_.string())});
        }
        dense_atlas_->at(hash).loadChunk();
        *dense_local_map_ += *(dense_atlas_->at(hash).chunk);

        if (!(sparse_atlas_->contains(hash))) {
            sparse_atlas_->insert({hash, Chunk(hash, vox_chunk_path_.string(), true, config_.voxel_size)});
        }
        sparse_atlas_->at(hash).loadChunk();
        *sparse_local_map_ += *(sparse_atlas_->at(hash).chunk);
    }

    if (!additional_hashes.has_value()) {return;}

    for (const auto &hash : additional_hashes.value()) {
        if (!(sparse_atlas_->contains(hash))) {
            sparse_atlas_->insert({hash, Chunk(hash, vox_chunk_path_.string())});
        }
        sparse_atlas_->at(hash).loadChunk();
        *sparse_local_map_ += *(sparse_atlas_->at(hash).chunk);
    }
}

/*
void Hylacomylus::deleteLastData()
{
    if (!(collection_history_->size() > 0)) {
        return;
    }

    deleteMappingResult(collection_history_->top());
    collection_history_->pop();
}

void Hylacomylus::deleteMappingResult(const MappingResult &result)
{
    // iterate over all data that was modified
    int counter {};
    for (const auto &hash : result.hashes) {
        // check if in memory
        if (!(atlas_->contains(hash))) {
            atlas_->insert({hash, Chunk(hash, chunk_path_.string())});
        }
        // load it up
        atlas_->at(hash).loadChunk();

        // loop through all points and extract points with matching ID
        pcl::PointIndices::Ptr outliers (new pcl::PointIndices());
        pcl::ExtractIndices<Point> extract;

        for (std::size_t i = 0; i < atlas_->at(hash).chunk->points.size(); i++) {
            if (atlas_->at(hash).chunk->points[i].collection_id == result.collection_id) {
                outliers->indices.push_back(i);
                counter++;
            }
        }

        if (!(outliers->indices.size() > 0)) {continue;}
        extract.setInputCloud(atlas_->at(hash).chunk);
        extract.setIndices(outliers);
        extract.setNegative(true);
        extract.filter(*(atlas_->at(hash).chunk));

        if (!(atlas_->at(hash).chunk->points.size() > 0)) {
            if (utils::checkFileExistence(atlas_->at(hash).getFileAddress())) {
                std::filesystem::remove(atlas_->at(hash).getFileAddress());
            }
        }
    }
}
*/

PointCloud::Ptr Hylacomylus::map()
{
    PointCloud::Ptr map (new PointCloud);
    *map = *dense_local_map_ + *sparse_local_map_;
    return map;
}

PointCloud::Ptr Hylacomylus::denseMap()
{
    return dense_local_map_;
}

PointCloud::Ptr Hylacomylus::sparseMap()
{
    return sparse_local_map_;
}

} // namespace hylacomylus
