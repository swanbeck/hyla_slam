/**
 * @file surface_repair_utils.hpp
 * @author Steven Swanbeck (steven.swanbeck@gmail.com)
 * @brief utilities to manipulate point cloud data
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#include "point_type.h"
#include <vector>
#include <variant>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/mls.h>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace surface_repair_utils {

// --------------------------------------------------------------------------------------
// & Point Types
// --------------------------------------------------------------------------------------

/** @typedef Point
 * @brief custom point type that is expected by all functions
 */
using Point = FabricMaintenance::Point;

/** @typedef PointCloud
 * @brief custom point cloud type that is expected by all functions
 */
using PointCloud = pcl::PointCloud<FabricMaintenance::Point>;

// --------------------------------------------------------------------------------------
// & Prototypes
// --------------------------------------------------------------------------------------

namespace transforms {

    /**
     * @brief rotates a cloud in place by some input angle about its z-axis
     * 
     * @param input pointer to cloud to be modified in place
     * @param angle angle of rotation to be applied about the z-axis
     */
    void rotateCloudZDegree (PointCloud::Ptr &input, const float &angle);

    /**
     * @brief converts a pose into an equivalent homogeneous transformation matrix
     * 
     * @param pose a pose with quaternion orientation and vector position
     * @return Eigen::Matrix4f 4x4 homogeneous transformation matrix
    */
    Eigen::Matrix4f pose2TransformationMatrix (const geometry_msgs::msg::Pose pose);

    /**
     * @brief converts a stamped pose into an equivalent homogeneous transformation matrix
     * 
     * @param pose a stamped pose with quaternion orientation and vector position
     * @return Eigen::Matrix4f 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4f poseStamped2TransformationMatrix (const geometry_msgs::msg::PoseStamped pose);

    /**
     * @brief converts a transform into an equivalent homogeneous transformation matrix
     * 
     * @param transform a transform with quaternion rotation and vector translation
     * @return Eigen::Matrix4f 4x4 homogeneous transfomation matrix
     */
    Eigen::Matrix4f transform2TransformationMatrix (const geometry_msgs::msg::Transform transform);

    /**
     * @brief converts a stamped transform into an equivalent homogeneous transformation matrix
     * 
     * @param transform a stamped transform with quaternion rotation and vector translation
     * @return Eigen::Matrix4f 4x4 homogeneous transfomation matrix
     */
    Eigen::Matrix4f transformStamped2TransformationMatrix (const geometry_msgs::msg::TransformStamped transform);

    /**
     * @brief converts a transform into an equivalent pose
     * 
     * @param transform a transform with quaternion rotation and vector translation
     * @return geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose transform2Pose (const geometry_msgs::msg::Transform transform);

    /**
     * @brief converts a stamped transform into an equivalent pose  
     * 
     * @param transform a stamped transform with quaternion rotation and vector translation
     * @return geometry_msgs::msg::Pose 
     */
    geometry_msgs::msg::Pose transformStamped2Pose (const geometry_msgs::msg::TransformStamped transform);

    /**
     * @brief converts a pose into an equivalent transform
     * 
     * @param pose a pose with quaternion orientation and vector position
     * @return geometry_msgs::msg::Transform 
     */
    geometry_msgs::msg::Transform pose2Transform (const geometry_msgs::msg::Pose pose);

    /**
     * @brief converts a stamped pose into an equivalent transform
     * 
     * @param pose a stamped pose with quaternion orientation and vector position
     * @return geometry_msgs::msg::Transform 
     */
    geometry_msgs::msg::Transform poseStamped2Transform (const geometry_msgs::msg::PoseStamped pose);

} // namespace transforms

namespace normals {

    /**
     * @brief generates surface normal vectors for an input point cloud
     * 
     * @param input pointer to cloud to be modified in place
     * @param normals_neighbors number of neighbors of each point used for normals calculation
     * 
     * @note a normal radius could be used instead of neighbors, but that makes the operation much more expensive and only makes a meaningful difference on sparse data
     * @warning input cloud is mutated
     */
    void generateSurfaceNormals (PointCloud::Ptr &input, const int &normals_neighbors=50);

} // namespace normals 

namespace registration {

    /**
     * @brief produces a transform to register two a source point cloud to some existing target
     * 
     * @param src pointer to the cloud to be registered (free in space) 
     * @param tgt pointer to the cloud to be used as reference for registration (fixed in space)
     * @param max_correspondence_distance maximum distance between any two correspondences to be considered for registration
     * @param iterations number of iterations of iterative closest point algorithm performed to produce the registration
     * @param use_normals indicates whether normal vectors are calculated and used for the registration process
     * @return Eigen::Matrix4f 4x4 homogeneous transformation matrix 
     * 
     * @note source cloud is NOT MUTATED (other than normals generation if specified); the returned transform must be applied to the cloud after to transform it, ie. pcl::transformPointCloud(*src, *ouput, transformation_matrix)
     * @note the ICP algorithm used by this function struggles to accurately register data if is it not generally already aligned; be aware of this limitation and make considerations to overcome it if necessary
     */
    Eigen::Matrix4f registerPointCloudsIterative (PointCloud::Ptr &src, PointCloud::Ptr &tgt, const float &max_correspondence_distance=0.5, const int &iterations=100, const bool &use_normals=false);

    // TODO keep trying to make a super robust registration function that is also as fast as possible
    Eigen::Matrix4f registerPointCloudsKeypoint (PointCloud::Ptr &src, PointCloud::Ptr &tgt);

} // namespace registration

namespace downsampling {

    /**
     * @brief downsamples a point cloud to some input voxel leaf size
     * 
     * @param input pointer to source point cloud
     * @param leaf_size voxel leaf size in meters
     * @return PointCloud::Ptr pointcloud pointer to downsampled cloud
     * 
     * @note input is not modified in place, unless the cloud is empty
     * @warning nonstandard/custom data fields in point clouds are generally lost during this operation; this allows the operation to be performed quickly but can be harmful for some applications. If data preservation is desired, instead use @fn losslessDownsamplePointCloud
     */
    PointCloud::Ptr lossyDownsamplePointCloud (const PointCloud::Ptr &input, const float &leaf_size);

    /**
     * @brief downsamples a point cloud to some input voxel leaf size while preserving normal vector and sensor position data
     * 
     * @param input pointer to source point cloud
     * @param leaf_size voxel leaf size in meters
     * @return std::variant<bool, PointCloud::Ptr> variant filled with either boolean false, indicating an unsuccessful operation, or a pointcloud pointer, indicating a successful operation
     * 
     * @note input is not modified in place, instead the output param is used and populated
     * @warning this operation is slow, as it performs expensive operations to preserve nonstandard information in point cloud data. If only spatial relations in your data need to be preserved, consider instead using @fn lossyDownsamplePointCloud
     */
    std::variant<bool, PointCloud::Ptr> losslessDownsamplePointCloud (const PointCloud::Ptr &input, const float &leaf_size, const std::variant<bool, PointCloud::Ptr> &reference, int downsample_neighbors=10);

} // namespace downsampling

namespace upsampling {
    /**
     * @brief upsamples a point cloud given various input parameters
     * 
     * @param input pointer to source point cloud
     * @param sampling_radius radius of neighbor search applied to each point for interpolating new points
     * @param step_size size of interpolation; while not truly, this parameters is the most similar to the "leaf_size" parameters in a downsampling function
     * @param search_radius radius of neighbor search applied to each point for interpolating new points
     * @param polynomial_order order of polynomial used for interpolation between points
     * @return PointCloud::Ptr upsampled cloud
     */
    PointCloud::Ptr lossyUpsamplePointCloud (const PointCloud::Ptr &input, const double sampling_radius=0.015, const double step_size=0.005, const double search_radius=0.03, const int polynomial_order=2);

    // TODO not yet implemented
    /**
     * @brief upsamples a point cloud
     * 
     * @param input pointer to source point cloud
     * @return std::variant<bool, PointCloud::Ptr> variant filled with either boolean false, indicating an unsuccessful operation, or a pointcloud pointer, indicating a successful operation
     * 
     * @note input is not modified in place, instead the output param is used and populated
     * @warning this operation is slow, as it performs expensive operations to preserve nonstandard information in point cloud data. If only spatial relations in your data need to be preserved, consider instead using @fn lossyDownsamplePointCloud
     */
    std::variant<bool, PointCloud::Ptr> losslessUpsamplePointCloud (const PointCloud::Ptr &input);

} // namespace upsampling

namespace clustering {

    /**
     * @brief given an input point cloud and empty vector of point clouds, clusters the input into smaller chunks and populates the vector in place
     * 
     * @param cloud pointer to the input cloud
     * @param clusters empty vector to be filled in place with the extracted clusters
     * @param min_cluster_size minimum size a cluster can be to be considered, in points
     * @param max_cluster_size maximum size a cluster can be to be considered, in points
     * @param num_neighbors number of closest neighbor points considered during clustering for each point
     * @param smoothness_threshold smoothness threshold within which points will be assigned to the same object (rather than segmented into a new one), in degrees
     * @param curvature_threshold curvature threshold within which points will be assigned to the same object (rather than segmented into a new one), in 1/m
     */
    std::variant<bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(PointCloud::Ptr cloud, std::vector<PointCloud::Ptr> &clusters, bool use_rgb=false, const int min_cluster_size=50, const int max_cluster_size=1e7, const int num_neighbors=10, const int smoothness_threshold=30, const int curvature_threshold=5);

} // namespace clustering

namespace io {

    /**
     * @brief checks if a files exists at a given file address
     * 
     * @param address full file address
     * @return true if file exists, false if not
     */
    bool checkFileExistence (const std::string &address);

} // namespace io

namespace statistics {

    /**
     * @brief Get the Cloud Centroid object
     * 
     * @param cloud input cloud
     * @return Eigen::Vector3f mass centroid of the cloud
     */
    Eigen::Vector3f getCloudCentroid(const PointCloud::Ptr &cloud);

} // namespace statistics

namespace misc {

    /**
     * @brief Splits an input string at a given delimiter character
     * 
     * @param str input string
     * @param delimiter char to serve at delimiter
     * @return std::vector<std::string> split string
     */
    std::vector<std::string> splitString(const std::string& str, char delimiter);

} // namespace misc

// --------------------------------------------------------------------------------------
// & Transforms
// --------------------------------------------------------------------------------------

namespace transforms {

void rotateCloudZDegree (PointCloud::Ptr &input, const float &angle) {
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();

    Ti(0,0) = cos(angle * M_PI / 180);
    Ti(0,1) = -1 * sin(angle * M_PI / 180);
    Ti(1,0) = sin(angle * M_PI / 180);
    Ti(1,1) = cos(angle * M_PI / 180);

    pcl::transformPointCloud(*input, *input, Ti);
}

Eigen::Matrix4f pose2TransformationMatrix (const geometry_msgs::msg::Pose pose) {
    Eigen::Quaternionf q;
    q.w() = pose.orientation.w;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            T(i, j) = R(i, j);
        }
    }

    T(0,3) = pose.position.x;
    T(1,3) = pose.position.y;
    T(2,3) = pose.position.z;

    return T;
}

Eigen::Matrix4f poseStamped2TransformationMatrix (const geometry_msgs::msg::PoseStamped pose) {
    return pose2TransformationMatrix(pose.pose);
}

Eigen::Matrix4f transform2TransformationMatrix (const geometry_msgs::msg::Transform transform) {
    Eigen::Quaternionf q;
    q.w() = transform.rotation.w;
    q.x() = transform.rotation.x;
    q.y() = transform.rotation.y;
    q.z() = transform.rotation.z;
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            T(i, j) = R(i, j);
        }
    }

    T(0,3) = transform.translation.x;
    T(1,3) = transform.translation.y;
    T(2,3) = transform.translation.z;

    return T;
}

Eigen::Matrix4f transformStamped2TransformationMatrix (const geometry_msgs::msg::TransformStamped transform) {
    return transform2TransformationMatrix(transform.transform);
}

geometry_msgs::msg::Pose transform2Pose (const geometry_msgs::msg::Transform transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation.w = transform.rotation.w;
    pose.orientation.x = transform.rotation.x;
    pose.orientation.y = transform.rotation.y;
    pose.orientation.z = transform.rotation.z;
    return pose;
}

geometry_msgs::msg::Pose transformStamped2Pose (const geometry_msgs::msg::TransformStamped transform) {
    return transform2Pose(transform.transform);
}

geometry_msgs::msg::Transform pose2Transform (const geometry_msgs::msg::Pose pose) {
    geometry_msgs::msg::Transform transform;
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation.w = pose.orientation.w;
    transform.rotation.x = pose.orientation.x;
    transform.rotation.y = pose.orientation.y;
    transform.rotation.z = pose.orientation.z;
    return transform;
}

geometry_msgs::msg::Transform poseStamped2Transform (const geometry_msgs::msg::PoseStamped pose) {
    return pose2Transform(pose.pose);
}

geometry_msgs::msg::Transform transformationMatrix2Transform (const Eigen::Matrix4f matrix) {
    geometry_msgs::msg::Transform transform;

    Eigen::Quaternionf q((Eigen::Matrix3f)matrix.block(0, 0, 3, 3));

    transform.rotation.w = q.w();
    transform.rotation.x = q.x();
    transform.rotation.y = q.y();
    transform.rotation.z = q.z();
    transform.translation.x = matrix(0, 3);
    transform.translation.y = matrix(1, 3);
    transform.translation.z = matrix(2, 3);
    return transform;
}

} // namespace transforms

// --------------------------------------------------------------------------------------
// & Normals
// --------------------------------------------------------------------------------------
namespace normals {

void generateSurfaceNormals (PointCloud::Ptr &input, const int &normals_neighbors) {
    if (input->points.size() < 1) {
        return;
    }

    std::cout << "[generateSurfaceNormals] Calculating normals for a cloud of size " << input->points.size() << "..." << std::endl;
    pcl::console::TicToc tt;
    tt.tic();

    pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(input);

    // ne.setRadiusSearch(normals_radius);
    ne.setKSearch(normals_neighbors);

    ne.setViewPoint(input->points[0].sensor_a, input->points[0].sensor_b, input->points[0].sensor_c);
    ne.compute(*cloud_normals);

    assert(input->points.size() == cloud_normals->points.size());

    for (std::size_t i = 0; i < input->points.size(); i++) {
        input->points[i].curvature = cloud_normals->points[i].curvature;
        input->points[i].normal_x = cloud_normals->points[i].normal_x;
        input->points[i].normal_y = cloud_normals->points[i].normal_y;
        input->points[i].normal_z = cloud_normals->points[i].normal_z;
    }

    std::cout << "[generateSurfaceNormals] >> Exiting: " << tt.toc() << "ms." << std::endl;
}

} // namspace normals

// --------------------------------------------------------------------------------------
// & Registration
// --------------------------------------------------------------------------------------

namespace registration {

class RegPointRepresentation : public pcl::PointRepresentation <Point>
{
    using pcl::PointRepresentation<Point>::nr_dimensions_;
public:
    RegPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const Point &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

Eigen::Matrix4f registerPointCloudsIterative (PointCloud::Ptr &src, PointCloud::Ptr &tgt, const float &max_correspondence_distance, const int &iterations, const bool &use_normals) {
    std::cout << "[registerPointCloudsIterative] Registering a cloud(source) of size " << src->points.size() << " to a cloud(target) of size " << tgt->points.size() << "..." << std::endl;
    pcl::console::TicToc tt;
    tt.tic();

    if (use_normals) {
        normals::generateSurfaceNormals(src);
    }

    if (tgt->points.size() < 1) {
        std::cout << "[registerPointCloudsIterative] >> Exiting: " << tt.toc() << "ms." << std::endl;
        return Eigen::Matrix4f::Identity();
    }

    PointCloud::Ptr src_downsampled (new PointCloud);
    PointCloud::Ptr tgt_downsampled (new PointCloud);
    src_downsampled = downsampling::lossyDownsamplePointCloud(src, 0.3);
    tgt_downsampled = downsampling::lossyDownsamplePointCloud(tgt, 0.3);

    pcl::IterativeClosestPointNonLinear<Point, Point> icp;
    icp.setTransformationEpsilon(1e-06);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    if (use_normals) {
        RegPointRepresentation point_representation;
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};
        point_representation.setRescaleValues (alpha);
        icp.setPointRepresentation(pcl::make_shared<const RegPointRepresentation>(point_representation));
    }
    icp.setInputSource(src_downsampled);
    icp.setInputTarget(tgt_downsampled);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
    icp.setMaximumIterations(2);

    PointCloud::Ptr final_cloud = src_downsampled;
    for (int i = 0; i < iterations; i++) {
        src_downsampled = final_cloud;
        icp.setInputSource(src_downsampled);
        icp.align(*final_cloud);

        Ti = icp.getFinalTransformation() * Ti;

        // std::cout << std::abs((icp.getLastIncrementalTransformation() - prev).sum()) << ", " << icp.getTransformationEpsilon() << ", " << icp.getFitnessScore() << std::endl;
        if (std::abs((icp.getLastIncrementalTransformation() - prev).sum()) < icp.getTransformationEpsilon()) {
            icp.setMaxCorrespondenceDistance(std::max(icp.getMaxCorrespondenceDistance() - 0.05, 0.02));
        }
        
        prev = icp.getLastIncrementalTransformation ();
    }

    std::cout << "Calibrated registration transform:\n" << Ti << std::endl;
    std::cout << "[registerPointCloudsIterative] >> Exiting: " << tt.toc() << "ms." << std::endl;

    return Ti;
}

} // namespace registration

// --------------------------------------------------------------------------------------
// & Downsampling
// --------------------------------------------------------------------------------------
namespace downsampling {

PointCloud::Ptr lossyDownsamplePointCloud (const PointCloud::Ptr &input, const float &leaf_size) {
    std::cout << "[lossyDownsamplePointCloud] Downsampling a cloud of size " << input->points.size() << " with leaf size " << leaf_size << "..." << std::endl;
    pcl::console::TicToc tt;
    tt.tic();

    // checking to make sure there is data to downsample
    if (!(input->points.size() > 0)) {
        std::cout << "[lossyDownsamplePointCloud] Input cloud is empty! Returning input: " << tt.toc() << "ms." << std::endl;
        return input;
    }

    PointCloud::Ptr output (new PointCloud);

    pcl::VoxelGrid<Point> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(input);
    grid.filter(*output);

    std::cout << "[lossyDownsamplePointCloud] >> Exiting, output cloud has size " << output->points.size() << ": " << tt.toc() << "ms." << std::endl;
    return output;
}

// ! the previous version of this function was SUPER computationally-expensive; the revised version may be faster, but this one should be used sparingly unless some large algorithmic improvement is made
std::variant<bool, PointCloud::Ptr> losslessDownsamplePointCloud (const PointCloud::Ptr &input, const float &leaf_size, const std::variant<bool, PointCloud::Ptr> &reference, int downsample_neighbors)
{
    std::cout << "[losslessDownsamplePointCloud] Downsampling a cloud of size " << input->points.size() << " with leaf size " << leaf_size << "..." << std::endl;
    pcl::console::TicToc tt;
    tt.tic();

    // checking to make sure there is data to downsample
    if (!(input->points.size() > 0)) {
        std::cout << "[losslessDownsamplePointCloud] Input cloud is empty! Returning input: " << tt.toc() << "ms." << std::endl;
        return input;
    }

    PointCloud::Ptr output (new PointCloud);

    // copy the cloud to a simpler type
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*input, *normal_cloud);

    // downsample it with data loss
    pcl::VoxelGrid<pcl::PointNormal> vox;
    vox.setInputCloud(normal_cloud);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointNormal>);
    vox.filter(*downsampled_cloud);

    // copy the downsampled normals cloud to the output
    pcl::copyPointCloud(*downsampled_cloud, *output);

    // search the original cloud at points and assign values accordingly
    pcl::KdTreeFLANN<Point> kdtree;

    // assign the cloud used for reference based on whether an explicit reference was provided; default to using the input if one was not
    PointCloud::Ptr reference_cloud (new PointCloud);
    if (std::holds_alternative<bool>(reference)) {
        pcl::copyPointCloud(*input, *reference_cloud);
    } else {
        pcl::copyPointCloud(*(std::get<PointCloud::Ptr>(reference)), *reference_cloud);
    }
    kdtree.setInputCloud(reference_cloud);

    // this is useful for tracking normal components to later be summed
    struct NormalsAccumulator {
        float x {};
        float y {};
        float z {};
        int n {};
        void accumulate (Point p) {
            x += p.normal_x;
            y += p.normal_y;
            z += p.normal_z;
            n++;
        }
        Eigen::Vector3f computeNormalAverage() {
            Eigen::Vector3f vec(x/n, y/n, z/n);
            return [&vec] {
                vec.normalize();
                return vec;
            }();
        }
    };

    struct RGBAccumulator {
        int R {};
        int G {};
        int B {};
        int n {};
        void accumulate (Point p) {
            R += p.r;
            G += p.g;
            B += p.b;
            n++;
        }
        std::array<int, 3> computeRGBAverage() {
            std::array<int, 3> rgb {
                static_cast<int>(static_cast<float>(R) / n),
                static_cast<int>(static_cast<float>(G) / n),
                static_cast<int>(static_cast<float>(B) / n)
            };
            return rgb;
        }
    };

    // rescaling the downsample neighbors
    if (downsample_neighbors > (int)(static_cast<float>(reference_cloud->points.size()) / 2)) {
        downsample_neighbors = ceil(static_cast<float>(reference_cloud->points.size()) / 2);
    }

    // iterate over the output cloud
    for (std::size_t i = 0; i < output->points.size(); i++) {

        // these will be filled by the search operation
        std::vector<int> point_idx_knn_search (downsample_neighbors);
        std::vector<float> point_knn_sqrd_distances (downsample_neighbors);

        // find nearest neighbors in reference cloud
        if (!(kdtree.nearestKSearch(output->points[i], downsample_neighbors, point_idx_knn_search, point_knn_sqrd_distances) > 0)) {
            continue;
        }
        
        // . get sensor position and normal vector information from all the neighbors of the point of interest

        // this is useful for tracking a sensor position and how many times it is seen within a cloud -> important for determining the side from which an area should be covered if it was imaged from several directions
        std::map<std::array<float, 3>, int> sensor_positions;

        NormalsAccumulator normals_accumulator;
        RGBAccumulator rgb_accumulator;
        
        // iterate over all searched points and accumulate data
        for (std::size_t j = 0; j < point_idx_knn_search.size(); j++) {

            // get the sensor position of the current point (rounding it here to two decimal places so so we don't duplicate positions that really should be the same)
            std::array<float, 3> sensor_position {
                std::round(100 * (*input)[point_idx_knn_search[j]].sensor_a) / 100,
                std::round(100 * (*input)[point_idx_knn_search[j]].sensor_b) / 100,
                std::round(100 * (*input)[point_idx_knn_search[j]].sensor_c) / 100
            };

            // add it to sensor_positions if it isn't there, increment otherwise
            if (sensor_positions.contains(sensor_position)) {
                sensor_positions.at(sensor_position)++;
            } else {
                sensor_positions.insert({sensor_position, 0});
            }

            // now accumulate the normal vector components
            normals_accumulator.accumulate((*input)[point_idx_knn_search[j]]);
            
            // // and also accumulate color
            // rgb_accumulator.accumulate((*input)[point_idx_knn_search[j]]);
        }

        // just a check to make sure all normals were properly accumulated
        assert(normals_accumulator.n == (int)(point_idx_knn_search.size()));

        // now find the highest frequency sensor position and assign it to the output point
        auto mode_sensor_position = std::max_element(
            std::begin(sensor_positions), std::end(sensor_positions),
            [] (const auto &pair1, const auto &pair2) {
                return pair1.second < pair2.second;
            }
        );
        output->points[i].sensor_a = mode_sensor_position->first.at(0);
        output->points[i].sensor_b = mode_sensor_position->first.at(1);
        output->points[i].sensor_c = mode_sensor_position->first.at(2);

        // and assign the average normal vector
        auto average_normal {normals_accumulator.computeNormalAverage()};
        output->points[i].normal_x = average_normal.x();
        output->points[i].normal_y = average_normal.y();
        output->points[i].normal_z = average_normal.z();

        // // and assign the RGB values
        // auto rgb {rgb_accumulator.computeRGBAverage()};
        // output->points[i].r = rgb.at(0);
        // output->points[i].g = rgb.at(1);
        // output->points[i].b = rgb.at(2);
    }

    std::cout << "[losslessDownsamplePointCloud] >> Exiting, output cloud has size " << output->points.size() << ": " << tt.toc() << "ms." << std::endl;
    return output;
}

} // namespace downsampling

// --------------------------------------------------------------------------------------
// & Upsampling
// --------------------------------------------------------------------------------------

namespace upsampling {

PointCloud::Ptr lossyUpsamplePointCloud (const PointCloud::Ptr &input, const double sampling_radius, const double step_size, const double search_radius, const int polynomial_order)
{
    std::cout << "[lossyUpsamplePointCloud] Upsampling a cloud of size " << input->points.size() << "..." << std::endl;
    pcl::console::TicToc tt;
    tt.tic();

    // checking to make sure there is data to upsample
    if (!(input->points.size() > 0)) {
        std::cout << "[lossyUpsamplePointCloud] Input cloud is empty! Returning input: " << tt.toc() << "ms." << std::endl;
        return input;
    }

    PointCloud::Ptr output (new PointCloud);

    // set up the upsampling
    pcl::search::KdTree<Point>::Ptr kd_tree (new pcl::search::KdTree<Point>());
    pcl::MovingLeastSquares<Point, Point> mls;

    double gauss_param {(double)std::pow(search_radius, 2)};
    mls.setComputeNormals(false);
    mls.setInputCloud(input);
    mls.setSearchMethod(kd_tree);
    mls.setSearchRadius(search_radius);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<Point, Point>::UpsamplingMethod::VOXEL_GRID_DILATION);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<Point, Point>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(sampling_radius);
    mls.setUpsamplingStepSize(step_size);
    mls.setPolynomialOrder(polynomial_order);
    mls.setSqrGaussParam(gauss_param);
    mls.setCacheMLSResults(true);
    mls.setNumberOfThreads(1);

    // compute it
    mls.process(*output);

    // adding the original points to the cloud to preserve the non-spatial information
    *output += *input;

    std::cout << "[lossyUpsamplePointCloud] >> Exiting, output cloud has size " << output->points.size() << ": " << tt.toc() << "ms." << std::endl;
    return output;
}

} // namespace upsampling

// --------------------------------------------------------------------------------------
// & Clustering
// --------------------------------------------------------------------------------------

namespace clustering {

std::variant<bool, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(PointCloud::Ptr cloud, std::vector<PointCloud::Ptr> &clusters, bool use_rgb, const int min_cluster_size, const int max_cluster_size, const int num_neighbors, const int smoothness_threshold, const int curvature_threshold)
{
    // handle case of empty input cloud
    if (!(cloud->points.size() > 0)) {return false;}

    // searching using just normals and xyz fields
    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::IndicesPtr indices (new std::vector<int>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr spatial_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *spatial_cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud, *normals);

    // region-growing segmentation
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(min_cluster_size);
    reg.setMaxClusterSize(max_cluster_size);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(num_neighbors);
    reg.setInputCloud(spatial_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(smoothness_threshold / 180.0 * M_PI);
    reg.setCurvatureThreshold(curvature_threshold);

    std::vector<pcl::PointIndices> cluster_indices;
    reg.extract(cluster_indices);

    // handle case where there are no clusters extracted
    if (!(cluster_indices.size() > 0)) {return false;}

    // build clusters and return
    for (std::size_t i = 0; i < cluster_indices.size(); i++) {
        PointCloud::Ptr cluster (new PointCloud);
        for (std::size_t j = 0; j < cluster_indices[i].indices.size(); j++) {
            cluster->push_back(cloud->points[cluster_indices[i].indices[j]]);
        }
        clusters.push_back(cluster);
    }

    // getting and publishing colored cloud for visualization purposes
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

    if (use_rgb) {
        for (const auto &cluster : clusters) {
            for (std::size_t i = 0; i < cluster->points.size(); i++) {
                pcl::PointXYZRGB p;
                p.x = cluster->points[i].x;
                p.y = cluster->points[i].y;
                p.z = cluster->points[i].z;
                p.r = cluster->points[i].r;
                p.g = cluster->points[i].g;
                p.b = cluster->points[i].b;
                colored_cloud->points.push_back(p);
            }
        }
    } else {
        colored_cloud = reg.getColoredCloud();
    }

    return colored_cloud;
}   

} // namespace clustering

// --------------------------------------------------------------------------------------
// & IO
// --------------------------------------------------------------------------------------

namespace io {

bool checkFileExistence (const std::string &address)
{
    if (FILE *file = fopen(address.c_str(), "r")) {
        fclose(file);
        return true;
    }
    return false;
}

} // namespace io

// --------------------------------------------------------------------------------------
// & Statistics
// --------------------------------------------------------------------------------------

namespace statistics {

Eigen::Vector3f getCloudCentroid(const PointCloud::Ptr &cloud)
{
    float x {};
    float y {};
    float z {};
    for (std::size_t i = 0; i < cloud->points.size(); i++) {
        x += cloud->points[i].x;
        y += cloud->points[i].y;
        z += cloud->points[i].z;
    }
    Eigen::Vector3f centroid(
        x / static_cast<int>(cloud->points.size()),
        y / static_cast<int>(cloud->points.size()),
        z / static_cast<int>(cloud->points.size())
    );
    return centroid;
}

} // namespace statistics

// --------------------------------------------------------------------------------------
// & Misc
// --------------------------------------------------------------------------------------

namespace misc {

std::vector<std::string> splitString(const std::string& str, char delimiter) {
    std::vector<std::string> result;
    std::string token;
    std::istringstream tokenStream(str);

    while (std::getline(tokenStream, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}

} // namespace misc

// --------------------------------------------------------------------------------------
// & FIN
// --------------------------------------------------------------------------------------
}
