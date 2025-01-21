#include "hyla_sam.hpp"

namespace hyla_sam {

HylaSam::HylaSam()
{
    HylaSam::initialize();
}

gtsam::Pose3 HylaSam::sophus2Gtsam(const Sophus::SE3d &pose)
{
    Eigen::Matrix3d rot {pose.rotationMatrix()};
    Eigen::Vector3d trans {pose.translation()};
    gtsam::Rot3 gt_rot {gtsam::Rot3(rot)};
    return gtsam::Pose3(gt_rot, trans);
}

void HylaSam::initialize()
{
    graph_ = gtsam::NonlinearFactorGraph();
    noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished());
    loop_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished());

    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    isam2_ = gtsam::ISAM2(isam2_params);

    gtsam::Pose3 prior_pose {gtsam::Pose3()};
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, prior_pose, noise_));
    initial_estimate_.insert(0, prior_pose);
}

void HylaSam::addOdomFactor(const Sophus::SE3d &relative_odom)
{
    auto t_minus_1 {initial_estimate_.keys().back()};

    // convert pose to gtsam::Pose3
    auto odom {HylaSam::sophus2Gtsam(relative_odom)};

    // add between factor
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(t_minus_1, t_minus_1 + 1, odom, noise_));

    // add initial_guess
    gtsam::Pose3 initial_pose = initial_estimate_.at<gtsam::Pose3>(t_minus_1).compose(odom);
    initial_estimate_.insert(t_minus_1 + 1, initial_pose);
}

void HylaSam::addLoopClosureFactor(const Sophus::SE3d &registration_pose, const int &old_scan_index)
{
    auto rel_pose {HylaSam::sophus2Gtsam(registration_pose)};
    auto new_scan_index {initial_estimate_.keys().back()};
    
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(old_scan_index, new_scan_index, rel_pose, loop_noise_));
}

gtsam::Values HylaSam::optimize()
{
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimate_);
    gtsam::Values optimized {optimizer.optimize()};

    // compare the pre and post

    // if (optimized.size() != initial_estimate_.size()) {
    //     return optimized;
    // }

    // for (size_t i = 0; i < optimized.size(); ++i) {
    //     gtsam::Pose3 init_pose = initial_estimate_.at<gtsam::Pose3>(i);
    //     gtsam::Pose3 opt_pose = optimized.at<gtsam::Pose3>(i);
    //     std::cout << "InitPose " << i << ": " << init_pose << std::endl;
    //     std::cout << "OptPose " << i << ": " << opt_pose << std::endl;
    // }

    return optimized;
}

gtsam::NonlinearFactorGraph HylaSam::getGraph()
{
    return graph_;
}

} // namespace hyla_sam
