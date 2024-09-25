#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>
#include <chrono>
// #include <iostream>
#include <optional>

#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/core/Registration.hpp"
#include "kiss_icp/core/Threshold.hpp"
#include "kiss_icp/core/VoxelHashMap.hpp"

namespace hylacomylus {

struct KissConfig {
    // map params
    double voxel_size = 1.0;
    double max_range = 100.0;
    double min_range = 5.0;
    int max_points_per_voxel = 20;

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;

    // registration params
    int max_num_iterations = 500;
    double convergence_criterion = 0.0001;
    int max_num_threads = 0;

    // motion compensation
    bool deskew = false;
}; // struct KissConfig

using Vector3dVector = std::vector<Eigen::Vector3d>;
using Vector3dVectorTuple = std::tuple<Vector3dVector, Vector3dVector>;

class HylaKiss
{
public:
    HylaKiss(const KissConfig &config);

public:
    Vector3dVectorTuple registerFrame(const std::vector<Eigen::Vector3d> &frame);

    Vector3dVectorTuple voxelize(const std::vector<Eigen::Vector3d> &frame) const;

    void setMap(const std::vector<Eigen::Vector3d> &map);

    std::vector<Eigen::Vector3d> localMap() const { return local_map_.Pointcloud(); };

    const kiss_icp::VoxelHashMap &voxelMap() const { return local_map_; };
    kiss_icp::VoxelHashMap &voxelMap() { return local_map_; };

    const Sophus::SE3d &pose() const { return last_pose_; }
    Sophus::SE3d &pose() { return last_pose_; }

    const Sophus::SE3d &delta() const { return last_delta_; }
    Sophus::SE3d &delta() { return last_delta_; }

    Eigen::Matrix4d extrapolateTransform(const Eigen::Matrix4d &T_initial, double delta_t_initial, double delta_t_new);

private:
    Sophus::SE3d last_pose_;
    Sophus::SE3d last_delta_;

    KissConfig config_;
    kiss_icp::Registration registration_;
    kiss_icp::VoxelHashMap local_map_;
    kiss_icp::AdaptiveThreshold adaptive_threshold_;

    std::unique_ptr<std::chrono::time_point<std::chrono::steady_clock>> previous_t_;
    std::unique_ptr<std::chrono::duration<double>> previous_dt_;


}; // class HylaKiss

} // namespace hylacomylus
