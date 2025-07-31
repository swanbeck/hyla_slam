#include "hyla_icp.hpp"
#include <iostream>
#include <cmath>

namespace hyla_icp {

HylaIcp::HylaIcp(const LocalizationConfig &config)
    : config_(config),
      registration_(config.max_num_iterations, config.convergence_criterion, config.max_num_threads),
      local_map_(config.voxel_size, config.max_range, config.max_points_per_voxel),
      adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range)
{}

Vector3dVectorTuple HylaIcp::registerFrame(const std::vector<Eigen::Vector3d> &frame)
{
    // preprocess the input cloud
    const auto &cropped_frame = kiss_icp::Preprocess(frame, config_.max_range, config_.min_range);

    // voxelize
    const auto &[source, frame_downsample] = voxelize(cropped_frame);

    // get adaptive threshold
    auto sigma = adaptive_threshold_.ComputeThreshold();

    // extrapolate last_delta_ (to remove constant timing assumption in constant velocity model)
    auto current_time {std::chrono::steady_clock::now()};

    std::optional<Sophus::SE3d> extrapolated_delta(std::nullopt);
    if (previous_t_ != nullptr) {
        auto dt {std::chrono::duration<double>(current_time - *previous_t_)};

        if (previous_dt_ != nullptr) {
            extrapolated_delta = HylaIcp::extrapolateTransform(last_delta_, previous_dt_->count(), dt.count(), sigma);
        }

        previous_dt_ = std::make_unique<std::chrono::duration<double>>(dt);
    }

    previous_t_ = std::make_unique<std::chrono::time_point<std::chrono::steady_clock>>(current_time);

    // compute inital guess for ICP
    const auto initial_guess = last_pose_ * extrapolated_delta.value_or(last_delta_);

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

Vector3dVectorTuple HylaIcp::voxelize(const std::vector<Eigen::Vector3d> &frame) const
{
    const auto voxel_size = config_.voxel_size;
    const auto frame_downsample = kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}

void HylaIcp::setMap(const std::vector<Eigen::Vector3d> &map)
{
    // save current map (to be reintroduced once new map has been set)
    auto previous_map {local_map_.Pointcloud()};

    // convert to VoxelHashMap and set to class variable
    const auto &[source, frame_downsample] = voxelize(map);
    local_map_.Clear();
    local_map_.Update(frame_downsample, last_pose_.translation());

    // now reintroduce previous data (useful to bias toward set map but also maintain flexibility for what robot sees online)
    local_map_.Update(previous_map, last_pose_.translation());

    adaptive_threshold_ = kiss_icp::AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

void HylaIcp::setPose(const Sophus::SE3d &pose)
{
    last_pose_ = pose;
    adaptive_threshold_ = kiss_icp::AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

Sophus::SE3d HylaIcp::extrapolateTransform(const Sophus::SE3d &T_initial, const double delta_t_initial, const double delta_t_new, double &sigma) {
    Eigen::Matrix3d R_initial {T_initial.rotationMatrix()};
    Eigen::Vector3d t_initial {T_initial.translation()};

    // make sure dts are not crazy far apart
    if (delta_t_initial <= 0 || delta_t_new <= 0) {
        return T_initial;
    }

    double dt_scalar {delta_t_new / delta_t_initial};

    // scaling up sigma so that it better responds to drops in messages
    if (dt_scalar > 1.0) {
        sigma += sigma * log(dt_scalar);
    }

    double max_c {2.0};
    if (dt_scalar > max_c) {
        dt_scalar = max_c;
    }

    // extrapolate translation
    Eigen::Vector3d t_new {t_initial * dt_scalar};

    // extrapolate rotation
    Eigen::AngleAxisd axis_angle(R_initial);
    double new_angle {axis_angle.angle() * dt_scalar};
    Eigen::AngleAxisd new_axis_angle(new_angle, axis_angle.axis());
    Eigen::Matrix3d R_new {new_axis_angle.toRotationMatrix()};

    // reconstruct the new transformation
    Sophus::SE3d T_new(R_new, t_new);
    
    return T_new;
}

} // namespace hyla_icp
