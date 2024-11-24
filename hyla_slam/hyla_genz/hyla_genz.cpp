#include "hyla_genz.hpp"
#include <iostream>
#include <cmath>

namespace hyla_genz {

HylaGenz::HylaGenz(const GenzConfig &config)
    : config_(config),
      registration_(config.max_num_iterations, config.convergence_criterion),
      local_map_(config.voxel_size, config.max_range, config.map_cleanup_radius, config.planarity_threshold, config.max_points_per_voxel),
      adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range)
{}

Vector3dVectorTuple HylaGenz::registerFrame(const std::vector<Eigen::Vector3d> &frame)
{
    // preprocess the input cloud
    const auto &cropped_frame = genz_icp::Preprocess(frame, config_.max_range, config_.min_range);

    // voxelize
    static double voxel_size = config_.voxel_size;

    const auto &[source, frame_downsample] = voxelize(cropped_frame, voxel_size);

    if (static_cast<int>(source.size()) > config_.max_points_per_voxelized_scan){
        voxel_size += 0.01;
    }
    else if (static_cast<int>(source.size()) < config_.min_points_per_voxelized_scan){
        voxel_size -= 0.01;
    }

    // get adaptive threshold
    const double sigma = getAdaptiveThreshold();

    // TODO consider adding back in delta_t extrapolation if this doesn't work great out of the box
    const auto prediction = getPredictionModel();
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    // run ICP
    // const auto new_pose = registration_.AlignPointsToMap(source, local_map_, initial_guess, 3.0 * sigma, sigma / 3.0);

    const auto &[new_pose, planar_points, non_planar_points] = registration_.RegisterFrame(source, local_map_, initial_guess, 3.0 * sigma, sigma / 3.0);

    // compute difference between prediction and actual estimate
    const auto model_deviation = initial_guess.inverse() * new_pose;

    // update step: threshold, local map, delta, and last pose
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    poses_.push_back(new_pose);

    // return the input raw scan and points used for registration
    return {frame, source};
}

Vector3dVectorTuple HylaGenz::voxelize(const std::vector<Eigen::Vector3d> &frame, const double &voxel_size) const
{
    const auto frame_downsample = genz_icp::VoxelDownsample(frame, voxel_size * 0.5);
    const auto source = genz_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample};
}

double HylaGenz::getAdaptiveThreshold() {
    if (!hasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Sophus::SE3d HylaGenz::getPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool HylaGenz::hasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

void HylaGenz::setMap(const std::vector<Eigen::Vector3d> &map)
{
    // save current map (to be reintroduced once new map has been set)
    auto previous_map {local_map_.Pointcloud()};

    // convert to VoxelHashMap and set to class variable
    const auto &[source, frame_downsample] = voxelize(map, config_.voxel_size);
    local_map_.Clear();
    local_map_.Update(frame_downsample, poses_.back().translation());

    // now reintroduce previous data (useful to bias toward set map but also maintain flexibility for what robot sees online)
    local_map_.Update(previous_map, poses_.back().translation());

    // TODO add in adpative threshold update here to make more robust to large changes that may be introduced when localization_map is added? 
    // adaptive_threshold_.UpdateModelDeviation();
    // or maybe it just makes sense to reset it altogether?
    adaptive_threshold_ = genz_icp::AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

void HylaGenz::setPose(const Sophus::SE3d &pose)
{
    poses_ = std::vector<Sophus::SE3d>();
    poses_.push_back(pose);
    // poses_.back() = pose;
    // TODO should last_delta be updated as well?
    // last_delta_ = ;
    adaptive_threshold_ = genz_icp::AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

Sophus::SE3d HylaGenz::extrapolateTransform(const Sophus::SE3d &T_initial, const double delta_t_initial, const double delta_t_new, double &sigma) {
    Eigen::Matrix3d R_initial {T_initial.rotationMatrix()};
    Eigen::Vector3d t_initial {T_initial.translation()};

    // make sure dts are not crazy far apart
    if (delta_t_initial <= 0 || delta_t_new <= 0) {
        // std::cout << "One or both delta_t values are less than or equal to zero!" << std::endl;
        return T_initial;
    }

    double dt_scalar {delta_t_new / delta_t_initial};

    // scaling up sigma so that it better responds to drops in messages
    if (dt_scalar > 1.0) {
        sigma += sigma * log(dt_scalar);
    }

    double max_c {2.0};
    if (dt_scalar > max_c) {
        // std::cout << "delta_t_new (" << delta_t_new << ") is more than " << max_c << "x delta_t_initial (" << delta_t_initial << "). This is likely to cause errors! Capping the dt at " << max_c << " the previous..." << std::endl;
        dt_scalar = max_c;

        // double c {max_c - log(max_c)};
        // dt_scalar = 1 + log(dt_scalar) + c; // note this is linear until max_c, then becomes logarithmic
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

} // namespace hyla_genz
