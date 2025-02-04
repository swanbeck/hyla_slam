#pragma once

#include <set>
#include <sstream>
#include <optional>
#include <filesystem>

#include "types.hpp"
#include "surface_repair_common/point_type.h"

namespace hylacomylus {

using Point = FabricMaintenance::Point;
using PointCloud = pcl::PointCloud<FabricMaintenance::Point>;

struct Chunk {
    const hash256_t id;
    const std::string dir;
    bool loaded;
    bool sparse;
    float leaf_size;
    PointCloud::Ptr chunk;

    Chunk(hash256_t handle, std::string root_dir, bool is_sparse=false, std::optional<float> leaf=std::nullopt) : id(handle), dir(root_dir), loaded(false), sparse(is_sparse), chunk(new PointCloud) {
        leaf_size = leaf.value_or(0.1);
    };

    std::string getFileAddress() {
        return std::filesystem::path(dir).append(to_hex_string(id) + ".pcd");
    }

    void add(const PointCloud::Ptr &addition) {
        *chunk += *addition;
    }

    void load() {
        if (loaded) {
            return;
        }
        
        if (std::filesystem::exists(Chunk::getFileAddress())) {
            pcl::io::loadPCDFile(Chunk::getFileAddress(), *chunk);
        }
        loaded = true;
    }

    void unload() {
        if (!(chunk->points.size() > 0)) {
            return;
        }

        if (!loaded) {
            if (std::filesystem::exists(Chunk::getFileAddress())) {
                PointCloud::Ptr existing (new PointCloud);
                pcl::io::loadPCDFile(Chunk::getFileAddress(), *existing);
                *chunk += *existing;
            }
        }

        if (sparse) {
            downsample();
        }

        chunk->height = 1;
        chunk->width = chunk->points.size();
        pcl::io::savePCDFileBinary(Chunk::getFileAddress(), *chunk);
        chunk = std::make_shared<PointCloud>();
        loaded = false;
    }

    void downsample() {
        pcl::VoxelGrid<Point> grid;
        grid.setInputCloud(chunk);
        grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        PointCloud::Ptr voxelized_cloud (new PointCloud);
        grid.filter(*voxelized_cloud);
        *chunk = *voxelized_cloud;
    }

}; // struct Chunk

} // namespace hylacomylus
