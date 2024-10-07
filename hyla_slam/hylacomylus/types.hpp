#pragma once

#include <string>
#include <set>
#include <Eigen/Dense>

namespace hylacomylus {

struct MappingConfig
{
    std::string fixed_frame;
    std::string robot_frame;
    
    int chunk_discretization;
    std::string chunk_load_dir;
    double half_side_length;

    bool active_mapping;
    bool persist_recent_chunks;
}; // struct MappingConfig

struct MappingResult
{
    std::uint32_t collection_id;
    std::set<std::uint64_t> hashes;
    MappingResult(std::uint32_t stamp, std::set<std::uint64_t> hashset) : collection_id(stamp), hashes(hashset) 
    {}
}; // struct MappingResult

} // namespace hylacomylus
