#pragma once

#include <string>
#include <set>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <stdexcept>

namespace hylacomylus {

struct hash256_t {
    std::uint64_t n0;
    std::uint64_t n1;
    std::uint64_t n2;
    std::uint64_t n3;

    hash256_t(std::uint64_t n0, std::uint64_t n1, std::uint64_t n2, std::uint64_t n3)
    : n0(n0), n1(n1), n2(n2), n3(n3) {}

    bool operator<(const hash256_t& other) const {
        if (n0 != other.n0) return n0 < other.n0;
        if (n1 != other.n1) return n1 < other.n1;
        if (n2 != other.n2) return n2 < other.n2;
        return n3 < other.n3;
    }
};

std::string to_hex_string(const hash256_t &hash);
hash256_t from_hex_string(const std::string &hex_str);

// std::ostream& operator<<(std::ostream& os, const hash256_t& hash);

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
    std::set<hash256_t> hashes;
    MappingResult(std::uint32_t stamp, std::set<hash256_t> hashset) : collection_id(stamp), hashes(hashset) 
    {}
}; // struct MappingResult

} // namespace hylacomylus
