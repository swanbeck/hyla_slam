#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace hyla_sam {

class HylaSam
{
public:
    HylaSam();

public:
    gtsam::Pose3 sophus2Gtsam(const Sophus::SE3d &pose);
    void initialize();
    void addOdomFactor(const Sophus::SE3d &relative_odom);

    void addRegistrationFactor();
    void addLoopClosureFactor();
    gtsam::Values optimize();

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::Values estimate_;
    gtsam::SharedNoiseModel noise_;
    gtsam::ISAM2 isam2_;

}; // class HylaSam

} // namespace hyla_sam
