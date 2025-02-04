#include "types.hpp"

namespace hylacomylus {

std::string to_hex_string(const hash256_t &hash) {
    std::stringstream ss;
    ss << std::setfill('0') << std::hex
        << std::setw(16) << hash.n0
        << std::setw(16) << hash.n1
        << std::setw(16) << hash.n2
        << std::setw(16) << hash.n3;
    return ss.str();
}

hash256_t from_hex_string(const std::string &hex_str) {
    if (hex_str.size() != 64) {
        throw std::invalid_argument("Hex string must be 64 characters long");
    }

    uint64_t n0, n1, n2, n3;
    std::stringstream ss0(hex_str.substr(0, 16));
    ss0 >> std::hex >> n0;
    std::stringstream ss1(hex_str.substr(16, 16));
    ss1 >> std::hex >> n1;
    std::stringstream ss2(hex_str.substr(32, 16));
    ss2 >> std::hex >> n2;
    std::stringstream ss3(hex_str.substr(48, 16));
    ss3 >> std::hex >> n3;

    return hash256_t(n0, n1, n2, n3);
}

} // namespace hylacomylus
