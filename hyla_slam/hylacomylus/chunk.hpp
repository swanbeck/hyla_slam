/**
 * @file types.hpp
 * @author Steven Swanbeck (steven.swanbeck@gmail.com)
 * @brief storage types for hylacomylus
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

// CPP
#include <set>
#include <sstream>
#include <optional>
#include <filesystem>
#include <boost/multiprecision/cpp_int.hpp>
using namespace boost::multiprecision;

// Custom
#include "surface_repair_common/point_type.h"
#include "utils.hpp"
// #include "surface_repair_common/utils.hpp"

namespace hylacomylus {

using Point = FabricMaintenance::Point;
using PointCloud = pcl::PointCloud<FabricMaintenance::Point>;

struct Chunk {
    const uint256_t id;
    const std::string dir;
    // const std::string frame;
    bool loaded;
    bool sparse;
    float leaf_size;
    PointCloud::Ptr chunk;

    Chunk(uint256_t handle, std::string root_dir, bool is_sparse=false, std::optional<float> leaf=std::nullopt) : id(handle), dir(root_dir), loaded(false), sparse(is_sparse), chunk(new PointCloud) {
        leaf_size = leaf.value_or(0.1);
    };

    std::string getFileAddress() {
        std::ostringstream oss;
        oss << id; // Convert uint256_t to a string
        return std::filesystem::path(dir).append(oss.str() + ".pcd");
    }
    bool isLoaded() {
        return loaded;
    }
    void addToChunk(const PointCloud::Ptr &addition) {
        *chunk += *addition;
    }
    void loadChunk() {
        if (loaded) {
            return;
        }
        
        if (utils::checkFileExistence(Chunk::getFileAddress())) {
            pcl::io::loadPCDFile(Chunk::getFileAddress(), *chunk);
        }
        loaded = true;
    }
    void unloadChunk() {
        if (!(chunk->points.size() > 0)) {
            return;
        }
        if (!loaded) { // if cloud is not loaded, we should check if stuff exists on disk
            if (utils::checkFileExistence(Chunk::getFileAddress())) { // if something does exist on disk, read it in and use it
                PointCloud::Ptr existing (new PointCloud);
                pcl::io::loadPCDFile(Chunk::getFileAddress(), *existing);
                *chunk += *existing;
            }
        }

        // if chunk is sparse, downsample it
        if (sparse) {
            downsampleChunk();
        }

        // finally, write the chunk to disk
        chunk->height = 1;
        chunk->width = chunk->points.size();
        pcl::io::savePCDFileBinary(Chunk::getFileAddress(), *chunk);
        chunk = std::make_shared<PointCloud>();
        loaded = false;
    }
    void downsampleChunk() {
        pcl::VoxelGrid<Point> sor;
        sor.setInputCloud(chunk);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        PointCloud::Ptr voxelized_cloud (new PointCloud);
        sor.filter(*voxelized_cloud);
        *chunk = *voxelized_cloud;
    }
}; // struct Chunk

} // namespace hylacomylus
