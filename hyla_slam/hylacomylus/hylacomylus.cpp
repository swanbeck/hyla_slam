#include "hylacomylus.hpp"

namespace hylacomylus {

Hylacomylus::Hylacomylus(const MappingConfig &config)
: config_(config), local_map_(new PointCloud)
{
    atlas_ = std::make_unique<std::map<std::uint64_t, Chunk>>();
    hasher_ = std::make_unique<ChunkHasher>(config_.chunk_discretization);
    collection_history_ = std::make_unique<std::stack<MappingResult>>();

    updateLocalMap(Sophus::SE3d());
}

Hylacomylus::~Hylacomylus()
{
    dumpMemoryData();
}

std::uint32_t Hylacomylus::generateTimeHash()
{
    std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
    auto durationSinceEpoch = currentTime.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch).count();
}

std::set<std::uint64_t> Hylacomylus::findLocalHashes(const Sophus::SE3d &robot_pose, const float &half_side_length)
{
    std::set<std::uint64_t> hashes;
    double increment {static_cast<double>(config_.chunk_discretization) / 2};
    for (double i = (robot_pose.translation().x() - half_side_length); i < (robot_pose.translation().x() + half_side_length); i += increment) {
        for (double j = (robot_pose.translation().y() - half_side_length); j < (robot_pose.translation().y() + half_side_length); j += increment) {
            for (double k = (robot_pose.translation().z() - half_side_length); k < (robot_pose.translation().z() + half_side_length); k += increment) {
                hashes.insert(hasher_->generateHash(Eigen::Vector3d(i, j, k)));
            }
        }
    }
    return hashes;
}

void Hylacomylus::rescopeStorage(const std::set<std::uint64_t> &hashes)
{
    // let's collect data associated with all hashes in memory
    for (const auto &hash : hashes) {
        // if atlas doesn't contain the hash, let's add it
        if (!(atlas_->contains(hash))) {
            atlas_->insert({hash, Chunk(hash, config_.chunk_load_dir)});
        }
        // then let's load it in
        atlas_->at(hash).loadChunk();
    }

    // then let's iterate over the data in memory and clean it up
    std::vector<std::uint64_t> erase_keys;
    for (auto &entry : *atlas_) {
        // if it's not in the hash set, let's get it ready to prune
        if (!(hashes.contains(entry.first))) {
            entry.second.unloadChunk();
            erase_keys.push_back(entry.first);
        }
    }

    // now actually prune the marked 
    for (const auto &key : erase_keys) {
        atlas_->erase(key);
    }
}

void Hylacomylus::update(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose)
{
    auto data_hashes {indexData(cloud, robot_pose, config_.active_mapping)};
    updateLocalMap(robot_pose, config_.persist_recent_chunks ? std::optional(data_hashes) : std::nullopt);
}

void Hylacomylus::updateLocalMap(const Sophus::SE3d &robot_pose, const std::optional<std::set<std::uint64_t>> &additional_hashes)
{
    auto hashes {findLocalHashes(robot_pose, config_.half_side_length)};
    if (additional_hashes.has_value()) {
        hashes.insert(additional_hashes.value().begin(), additional_hashes.value().end());
    }
    rescopeStorage(hashes);
    composeLocalMap(hashes);
}

std::set<std::uint64_t> Hylacomylus::indexData(PointCloud::Ptr &cloud, const Sophus::SE3d &robot_pose, const bool &add_data)
{
    std::uint32_t collection_hash {Hylacomylus::generateTimeHash()};
    std::set<std::uint64_t> data_hashes;

    // assign all points to an entry in atlas
    for (std::size_t i = 0; i < cloud->points.size(); i++) {

        std::uint64_t hash {hasher_->generateHash(cloud->points[i])};
        if (!(data_hashes.contains(hash))) {
            data_hashes.insert(hash);
        }

        if (add_data) {
            // stamp all these points with the updated base localization before they are indexed into atlas
            cloud->points[i].collection_id = collection_hash;
            cloud->points[i].sensor_a = robot_pose.translation().x();
            cloud->points[i].sensor_b = robot_pose.translation().y();
            cloud->points[i].sensor_c = robot_pose.translation().z();

            // add an entry to atlas if it doesn't yet exist
            if (!(atlas_->contains(hash))) {
                atlas_->insert({hash, Chunk(hash, config_.chunk_load_dir)});
                atlas_->at(hash).loadChunk();
            }

            // add the point to the corresponding atlas entry
            atlas_->at(hash).chunk->points.push_back(cloud->points[i]);
        }
    }

    if (add_data) {
        collection_history_->push(MappingResult(collection_hash, data_hashes));
    }

    return data_hashes;
}

void Hylacomylus::dumpMemoryData()
{
    for (auto &entry : *atlas_) {
        entry.second.unloadChunk();
    }
}

void Hylacomylus::composeLocalMap(const std::set<std::uint64_t> &hashes)
{
    local_map_->clear();

    for (const auto &hash : hashes) {
        if (!(atlas_->contains(hash))) {
            atlas_->insert({hash, Chunk(hash, config_.chunk_load_dir)});
        }
        atlas_->at(hash).loadChunk();
        *local_map_ += *(atlas_->at(hash).chunk);
    }
}

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
            atlas_->insert({hash, Chunk(hash, config_.chunk_load_dir)});
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

PointCloud::Ptr Hylacomylus::map()
{
    return local_map_;
}

} // namespace hylacomylus
