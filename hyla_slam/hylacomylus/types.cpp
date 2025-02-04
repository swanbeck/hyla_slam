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
    if (hex_str.size() != 64) {  // Ensure the hex string has 64 characters (256 bits)
        throw std::invalid_argument("Hex string must be 64 characters long");
    }

    std::uint64_t n0, n1, n2, n3;

    std::stringstream ss(hex_str);
    ss >> std::hex >> n0
        >> std::hex >> n1
        >> std::hex >> n2
        >> std::hex >> n3;

    return hash256_t(n0, n1, n2, n3);
}

} // namespace hylacomylus
