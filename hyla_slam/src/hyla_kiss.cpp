#include "hyla_slam/hyla_kiss.hpp"

namespace hylacomylus {

HylaKiss::HylaKiss(const KissConfig &config)
    : config_(config),
      registration_(config.max_num_iterations, config.convergence_criterion, config.max_num_threads),
      local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
      adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range)
{}

Vector3dVectorTuple HylaKiss::registerFrame(const std::vector<Eigen::Vector3d> &frame)
{
    // preprocess the input cloud
    const auto &cropped_frame = kiss_icp::Preprocess(frame, config_.max_range, config_.min_range);

    // voxelize
    const auto &[source, frame_downsample] = voxelize(cropped_frame);

    // get adaptive threshold
    const auto sigma = adaptive_threshold_.ComputeThreshold();

    // compute inital guess for ICP
    const auto initial_guess = last_pose_ * last_delta_;

    // run ICP
    const auto new_pose = registration_.AlignPointsToMap(source, local_map_, initial_guess, 3.0 * sigma, sigma / 3.0);

    // compute difference between prediction and actual estimate
    const auto model_deviation = initial_guess.inverse() * new_pose;

    // update step: threshold, local map, delta, and last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    last_delta_ = last_pose_.inverse() * new_pose;
    
    last_pose_ = new_pose;

    // return the input raw scan and points used for registration
    return {frame, source};
}

Vector3dVectorTuple HylaKiss::voxelize(const std::vector<Eigen::Vector3d> &frame) const
{
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}

void HylaKiss::setMap(const std::vector<Eigen::Vector3d> &map)
{
    // convert to VoxelHashMap and set to class variable
    const auto &[source, frame_downsample] = voxelize(map);

    local_map_.Clear();

    local_map_.Update(frame_downsample, last_pose_.translation());
}

} // namespace hylacomylus
