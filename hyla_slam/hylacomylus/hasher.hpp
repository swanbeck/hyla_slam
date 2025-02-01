#pragma once

#include <cstdint>
#include <array>
#include <map>
#include <Eigen/Dense>
#include <boost/multiprecision/cpp_int.hpp>

#include "chunk.hpp"

using namespace boost::multiprecision;

namespace hylacomylus {

class Hasher {
public:
    Hasher(int discretization_size) 
    {
        discretization = discretization_size;
        hash_signs = buildHashSigns();
        sign_hashes = buildSignHashes();
    }

    uint256_t generateHash(const Point &point)
    {
        return generateHash(Eigen::Vector3d(point.x, point.y, point.z));
    }

    uint256_t generateHash(const Eigen::Vector3d &point) 
    {
        std::uint8_t n0 {sign_hashes[{valueSign(point.x()), valueSign(point.y()), valueSign(point.z())}]};
        std::uint16_t n1 = valueMagnitude(point.x());
        std::uint16_t n2 = valueMagnitude(point.y());
        std::uint16_t n3 = valueMagnitude(point.z());

        uint256_t hash = static_cast<uint256_t>(n0) |
                            (static_cast<uint256_t>(n1) << 16) | 
                            (static_cast<uint256_t>(n2) << 32) | 
                            (static_cast<uint256_t>(n3) << 48);
        return hash;
    }

    Eigen::Vector3d parseHash(const uint256_t &hash)
    {
        Eigen::Vector3d p;
        std::uint8_t n0 {static_cast<std::uint8_t>(hash & 0xFF)};
        std::array<std::int8_t, 3> signs {hash_signs[n0]};
        p.x() = static_cast<float>(static_cast<std::uint16_t>((hash >> 16) & 0xFF)) * signs[0] * discretization;
        p.y() = static_cast<float>(static_cast<std::uint16_t>((hash >> 32) & 0xFF)) * signs[1] * discretization;
        p.z() = static_cast<float>(static_cast<std::uint16_t>((hash >> 48) & 0xFF)) * signs[2] * discretization;
        return p;
    }

private:
    int discretization;
    std::map<std::uint8_t, std::array<std::int8_t, 3>> hash_signs;
    std::map<std::array<std::int8_t, 3>, std::uint8_t> sign_hashes;

    std::map<std::uint8_t, std::array<std::int8_t, 3>> buildHashSigns() 
    {
        std::map<std::uint8_t, std::array<std::int8_t, 3>> hashSigns;
        hashSigns[0] = {1, 1, 1};
        hashSigns[1] = {-1, 1, 1};
        hashSigns[2] = {1, -1, 1};
        hashSigns[3] = {1, 1, -1};
        hashSigns[4] = {-1, -1, 1};
        hashSigns[5] = {-1, 1, -1};
        hashSigns[6] = {1, -1, -1};
        hashSigns[7] = {-1, -1, -1};
        return hashSigns;
    }

    std::map<std::array<std::int8_t, 3>, std::uint8_t> buildSignHashes() 
    {
        std::map<std::array<std::int8_t, 3>, std::uint8_t> signHashes;
        signHashes[{1, 1, 1}] = 0;
        signHashes[{-1, 1, 1}] = 1;
        signHashes[{1, -1, 1}] = 2;
        signHashes[{1, 1, -1}] = 3;
        signHashes[{-1, -1, 1}] = 4;
        signHashes[{-1, 1, -1}] = 5;
        signHashes[{1, -1, -1}] = 6;
        signHashes[{-1, -1, -1}] = 7;
        return signHashes;
    }

    std::int8_t valueSign(float value) 
    {
        if (value < 0) {
            return -1;
        }
        return 1;
    }

    std::uint16_t valueMagnitude(float value) 
    {
        return static_cast<std::uint16_t>(std::abs(value) / discretization);
    }
}; // class Hasher

}
