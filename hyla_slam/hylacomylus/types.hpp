#pragma once

#include <string>
#include <set>
#include <Eigen/Dense>

#include <boost/multiprecision/cpp_int.hpp>
using namespace boost::multiprecision;

namespace hylacomylus {

struct MappingConfig
{
    std::string data_dir;
    
    bool active_mapping;
    int chunk_discretization;
    bool persist_recent_chunks;
    int scan_memory_horizon;

    double dense_map_radius;
    double sparse_map_radius;

    int max_points_per_dense_chunk;
    int max_points_per_sparse_chunk;
    float sparse_voxel_size;

    bool save_dense_scans;
    bool save_sparse_scans;
    bool maintain_dense_chunks;
    bool maintain_sparse_chunks;
}; // struct MappingConfig

struct MappingResult
{
    std::uint32_t collection_id;
    std::set<uint256_t> hashes;
    MappingResult(std::uint32_t stamp, std::set<uint256_t> hashset) : collection_id(stamp), hashes(hashset) 
    {}
}; // struct MappingResult

} // namespace hylacomylus
