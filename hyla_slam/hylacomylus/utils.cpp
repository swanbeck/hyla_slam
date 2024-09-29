#include "utils.hpp"

namespace hylacomylus {

namespace utils {

bool checkFileExistence (const std::string &address)
{
    if (FILE *file = fopen(address.c_str(), "r")) {
        fclose(file);
        return true;
    }
    return false;
}

} // namespace utils

} // namespace hylacomylus
