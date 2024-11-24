#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>
#include <chrono>
#include <optional>
// #include <iostream>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/Threshold.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace hyla_genz {

struct GenzConfig {
    // map params
    double voxel_size = 0.25;
    double max_range = 100.0;
    double min_range = 0.5;
    int max_points_per_voxel = 1;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    // registration params
    int max_num_iterations = 150;
    double convergence_criterion = 0.0001;
    int max_num_threads = 0;

    // genz params
    double map_cleanup_radius = 50.0;
    double planarity_threshold = 0.12;
    int max_points_per_voxelized_scan = 2000;
    int min_points_per_voxelized_scan = 1300;

    // motion compensation
    bool deskew = false;
}; // struct GenzConfig

using Vector3dVector = std::vector<Eigen::Vector3d>;
using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

class HylaGenz
{
public:
    HylaGenz(const GenzConfig &config);

public:
    Vector3dVectorTuple registerFrame(const std::vector<Eigen::Vector3d> &frame);

    Vector3dVectorTuple voxelize(const std::vector<Eigen::Vector3d> &frame, const double &voxel_size) const;

    void setMap(const std::vector<Eigen::Vector3d> &map);

    void setPose(const Sophus::SE3d &pose);

    std::vector<Eigen::Vector3d> localMap() const { return local_map_.Pointcloud(); };

    const genz_icp::VoxelHashMap &voxelMap() const { return local_map_; };
    genz_icp::VoxelHashMap &voxelMap() { return local_map_; };

    const Sophus::SE3d &pose() const { return poses_.back(); }
    Sophus::SE3d &pose() { return poses_.back(); }

    Sophus::SE3d extrapolateTransform(const Sophus::SE3d &T_initial, const double delta_t_initial, const double delta_t_new, double &sigma);

    double getAdaptiveThreshold();
    Sophus::SE3d getPredictionModel() const;
    bool hasMoved();

private:
    GenzConfig config_;
    genz_icp::Registration registration_;
    genz_icp::VoxelHashMap local_map_;
    genz_icp::AdaptiveThreshold adaptive_threshold_;

    std::unique_ptr<std::chrono::time_point<std::chrono::steady_clock>> previous_t_;
    std::unique_ptr<std::chrono::duration<double>> previous_dt_;

    std::vector<Sophus::SE3d> poses_;

}; // class HylaGenz

} // namespace hyla_genz
