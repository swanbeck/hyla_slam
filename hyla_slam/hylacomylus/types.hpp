#pragma once

#include <string>
#include <set>
#include <Eigen/Dense>

#include <boost/multiprecision/cpp_int.hpp>
using namespace boost::multiprecision;

namespace hylacomylus {

struct MappingConfig
{
    std::string fixed_frame;
    std::string robot_frame;
    
    int chunk_discretization;
    std::string data_dir;
    double half_side_length;

    bool active_mapping;
    bool persist_recent_chunks;
    int recent_scan_memory;

    float voxel_size;
}; // struct MappingConfig

struct MappingResult
{
    std::uint32_t collection_id;
    std::set<uint256_t> hashes;
    MappingResult(std::uint32_t stamp, std::set<uint256_t> hashset) : collection_id(stamp), hashes(hashset) 
    {}
}; // struct MappingResult

} // namespace hylacomylus
