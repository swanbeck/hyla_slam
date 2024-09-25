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

    // TODO add timing element here
    // get current time
    auto current_time {std::chrono::steady_clock::now()};

    std::optional<Sophus::SE3d> extrapolated_delta(std::nullopt);
    if (previous_t_ != nullptr) {

        auto dt {std::chrono::duration<double>(current_time - *previous_t_)};

        if (previous_dt_ != nullptr) {
            auto extrapolated_transform {HylaKiss::extrapolateTransform(last_delta_.matrix(), previous_dt_->count(), dt.count())};

            Sophus::SE3d updated_delta(extrapolated_transform.block<3, 3>(0, 0), extrapolated_transform.block<3, 1>(0, 3));
            extrapolated_delta = updated_delta;
        }

        // compute previous_dt_ (not useful until next iteration)
        previous_dt_ = std::make_unique<std::chrono::duration<double>>(dt);
    }

    // save time for next iteration
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

    // TODO add in adpative threshold update here to make more robust to large changes that may be introduced when localization_map is added? 
    // adaptive_threshold_.UpdateModelDeviation();
    // or maybe it just makes sense to reset it altogether?
    adaptive_threshold_ = kiss_icp::AdaptiveThreshold(config_.initial_threshold, config_.min_motion_th, config_.max_range);
}

Eigen::Matrix4d HylaKiss::extrapolateTransform(const Eigen::Matrix4d &T_initial, double delta_t_initial, double delta_t_new) {
    // Extract the rotation matrix (top-left 3x3 part) and translation vector (top-right 3x1 part)
    Eigen::Matrix3d R_initial {T_initial.block<3, 3>(0, 0)};
    Eigen::Vector3d t_initial {T_initial.block<3, 1>(0, 3)};

    // Compute translation velocity
    Eigen::Vector3d v_translation {t_initial / delta_t_initial};
    
    // Compute angular velocity (using matrix logarithm to get angular velocity in axis-angle form)
    Eigen::AngleAxisd axis_angle(R_initial);

    // std::cout << "dt_prev: " << delta_t_initial << ", dt_new: " << delta_t_new << "..." << std::endl;

    // std::cout << "Angle:\n" << axis_angle.angle() << "\nAxis:\n" << axis_angle.axis() << "\n\n" << std::endl;

    // Extrapolate translation
    Eigen::Vector3d t_new {v_translation * delta_t_new};

    // Extrapolate rotation
    double new_angle {axis_angle.angle() * delta_t_new / delta_t_initial};
    Eigen::AngleAxisd new_axis_angle(new_angle, axis_angle.axis());
    Eigen::Matrix3d R_new {new_axis_angle.toRotationMatrix()};

    // std::cout << "Updated Angle:\n" << new_axis_angle.angle() << "\nUpdated Axis:\n" << new_axis_angle.axis() << "\n\n\n" << std::endl;
    
    // Reconstruct the new transformation matrix
    Eigen::Matrix4d T_new {Eigen::Matrix4d::Identity()};
    T_new.block<3, 3>(0, 0) = R_new;
    T_new.block<3, 1>(0, 3) = t_new;
    
    return T_new;
}

} // namespace hylacomylus
