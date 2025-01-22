#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
#include <iostream>

#include "hylacomylus/chunk.hpp"

// Voxel structure for maintaining occupancy probabilities
struct Voxel {
    float probability = 0.5f; // Initial occupancy probability
};

// Hash function for 3D voxel indexing
struct VoxelHash {
    std::size_t operator()(const Eigen::Vector3i& key) const {
        return std::hash<int>()(key.x()) ^ std::hash<int>()(key.y()) ^ std::hash<int>()(key.z());
    }
};

// Ray traversal using 3D DDA
void raycast(const Eigen::Vector3f& origin, const Eigen::Vector3f& point, float voxel_size,
             std::unordered_map<Eigen::Vector3i, Voxel, VoxelHash>& occupancy_grid) {
    Eigen::Vector3f direction = (point - origin).normalized();
    Eigen::Vector3f ray = origin;
    
    // Convert origin and point to voxel indices
    Eigen::Vector3i voxel_origin = (origin / voxel_size).array().floor().cast<int>();
    Eigen::Vector3i voxel_point = (point / voxel_size).array().floor().cast<int>();
    Eigen::Vector3i voxel = voxel_origin;

    Eigen::Vector3f tMax, tDelta;
    Eigen::Vector3i step;

    for (int i = 0; i < 3; ++i) {
        if (direction[i] > 0) {
            step[i] = 1;
            tMax[i] = ((voxel[i] + 1) * voxel_size - origin[i]) / direction[i];
            tDelta[i] = voxel_size / direction[i];
        } else if (direction[i] < 0) {
            step[i] = -1;
            tMax[i] = (voxel[i] * voxel_size - origin[i]) / direction[i];
            tDelta[i] = -voxel_size / direction[i];
        } else {
            step[i] = 0;
            tMax[i] = std::numeric_limits<float>::max();
            tDelta[i] = std::numeric_limits<float>::max();
        }
    }

    // std::cout << "step: " << step[0] << ", " << step[1] << ", " << step[2] << "\ntMax: " << tMax[0] << ", " << tMax[1] << ", " << tMax[2] << "\ntDelta: " << tDelta[0] << ", " << tDelta[1] << ", " << tDelta[2] << std::endl;

    // std::cout << "\tvoxel: " << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << std::endl;
    // std::cout << "\tvoxpt: " << voxel_point[0] << ", " << voxel_point[1] << ", " << voxel_point[2] << std::endl;

    // Ray traversal loop
    const int max_iterations = 30; // Safeguard against infinite loops
    int iterations = 0;

    if ((voxel_point - voxel).norm() < 100) {
        while (voxel != voxel_point) {
            // Update occupancy probabilities for the current voxel
            occupancy_grid[voxel].probability -= 0.1f; // Decrease probability for ray traversal

            // Find the axis with the smallest tMax value
            int min_axis = 0;
            if (tMax[1] < tMax[0]) min_axis = 1;
            if (tMax[2] < tMax[min_axis]) min_axis = 2;

            // Move to the next voxel along the selected axis
            voxel[min_axis] += step[min_axis];
            tMax[min_axis] += tDelta[min_axis];

            // Check for overshooting
            if (++iterations > max_iterations) {
                // std::cerr << "Ray traversal exceeded maximum iterations. Possible error in termination condition." << std::endl;
                break;
            }
        }
    }

    // // Traverse the ray
    // int counter {};
    // while (voxel != voxel_point) {
    //     occupancy_grid[voxel].probability -= 0.01f; // Decrease probability for ray traversal
    //     int min_axis = (tMax.x() < tMax.y()) ? ((tMax.x() < tMax.z()) ? 0 : 2) : ((tMax.y() < tMax.z()) ? 1 : 2);
    //     voxel[min_axis] += step[min_axis];
    //     tMax[min_axis] += tDelta[min_axis];
    //     ++counter;
    //     std::cout << "\t\tvoxel: " << voxel[0] << ", " << voxel[1] << ", " << voxel[2] << std::endl;
    //     // std::cout << "\t" << counter << ": " << min_axis << ", " << voxel[min_axis] << ", " << tMax[min_axis] << ", " << std::endl;
    // }

    // std::cout << "Followed ray to end (" << iterations << ")" << std::endl;

    // Mark the voxel containing the point
    occupancy_grid[voxel_point].probability += 0.01f; // Increase probability for point
}

int main() {
    hylacomylus::PointCloud::Ptr cloud(new hylacomylus::PointCloud());
    pcl::io::loadPCDFile("/home/darwin/config/data/cloud.pcd", *cloud);

    std::cout << "Read in cloud (" << cloud->points.size() << " points)!" << std::endl;

    float voxel_size = 0.3f; // Set your voxel resolution
    std::unordered_map<Eigen::Vector3i, Voxel, VoxelHash> occupancy_grid;

    int counter {};
    for (const auto& point : cloud->points) {
        Eigen::Vector3f point_pos(point.x, point.y, point.z);
        Eigen::Vector3f sensor_origin(point.sensor_a, point.sensor_b, point.sensor_c); // Set the sensor origin
        
        raycast(sensor_origin, point_pos, voxel_size, occupancy_grid);

        if (counter % 100000 == 0) {
            std::cout << "Completed " << counter << "/" << cloud->points.size() << " operations..." << std::endl;
        }
        ++counter;
    }

    // Filter the cloud based on occupancy probabilities
    hylacomylus::PointCloud::Ptr filtered_cloud(new hylacomylus::PointCloud());
    for (const auto& point : cloud->points) {
        Eigen::Vector3i voxel_idx = (Eigen::Vector3f(point.x, point.y, point.z) / voxel_size).cast<int>();
        if (occupancy_grid[voxel_idx].probability > 0.7f) { // Threshold for occupied
            filtered_cloud->points.push_back(point);
        }
    }

    std::cout << "Filtered cloud size: " << filtered_cloud->size() << std::endl;

    // save to file
    pcl::io::savePCDFileBinary("/home/darwin/config/data/filtered.pcd", *filtered_cloud);

    return 0;
}
