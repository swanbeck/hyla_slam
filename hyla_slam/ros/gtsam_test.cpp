#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <iostream>

int main() {
    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.01; // Threshold for relinearization
    isam2Params.relinearizeSkip = 1;        // Relinearize every iteration
    gtsam::ISAM2 isam2(isam2Params);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;

    // Add a prior factor at the first pose
    gtsam::Pose3 priorPose = gtsam::Pose3(); // Identity pose
    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3).finished());
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorPose, noise));

    // Add the first pose to the initial estimate
    initialEstimate.insert(0, priorPose);

    // Example odometry loop
    int num_poses {10};
    for (int t = 1; t < num_poses; ++t) {
        // Assume you have odometry as gtsam::Pose3 odometry (relative pose)
        // gtsam::Pose3 odometry = getOdometryReading(t); // Replace with actual data
        auto odometry {gtsam::Pose3()};
        gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.01, 0.01, 0.01).finished());
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(t - 1, t, odometry, noise));
        
        // Add an initial guess for the new pose
        gtsam::Pose3 initialPose = initialEstimate.at<gtsam::Pose3>(t - 1).compose(odometry);
        initialEstimate.insert(t, initialPose);
    }

    // if (detectLoopClosure(currentPoseIndex, loopPoseIndex)) {
    //     gtsam::Pose3 loopClosureConstraint = getLoopClosureConstraint(currentPoseIndex, loopPoseIndex);
    //     gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(
    //         (gtsam::Vector(6) << 0.2, 0.2, 0.2, 0.02, 0.02, 0.02).finished());
    //     graph.add(gtsam::BetweenFactor<gtsam::Pose3>(loopPoseIndex, currentPoseIndex, loopClosureConstraint, noise));
    // }

    // Update the iSAM2 solver with the new graph and estimates
    isam2.update(graph, initialEstimate);

    // Retrieve the optimized values
    gtsam::Values optimizedValues = isam2.calculateEstimate();

    // Clear the graph and initial estimate for the next iteration
    graph.resize(0);
    initialEstimate.clear();

    for (size_t i = 0; i < optimizedValues.size(); ++i) {
        gtsam::Pose3 pose = optimizedValues.at<gtsam::Pose3>(i);
        std::cout << "Pose " << i << ": " << pose << std::endl;
    }

    return 0;
}
