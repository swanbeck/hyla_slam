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

// Custom
#include "surface_repair_common/point_type.h"
#include "utils.hpp"
// #include "surface_repair_common/utils.hpp"

namespace hylacomylus {

using Point = FabricMaintenance::Point;
using PointCloud = pcl::PointCloud<FabricMaintenance::Point>;

struct Chunk {
    const std::uint64_t id;
    const std::string dir;
    // const std::string frame;
    bool loaded;
    PointCloud::Ptr chunk;

    Chunk(std::uint64_t handle, std::string root_dir) : id(handle), dir(root_dir), loaded(false), chunk(new PointCloud) {};

    std::string getFileAddress() {
        return dir + std::to_string(id) + ".pcd";
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
        // finally, write the chunk to disk
        chunk->height = 1;
        chunk->width = chunk->points.size();
        pcl::io::savePCDFileBinary(Chunk::getFileAddress(), *chunk);
        loaded = false;
    }
}; // struct Chunk

} // namespace hylacomylus
