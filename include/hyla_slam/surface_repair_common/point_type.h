/**
 * @file point_type.h
 * @author Steven Swanbeck (steven.swanbeck@gmail.com)
 * @brief custom point type declaration and registrations for surface repair data
 * @version 0.1
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/instantiate.hpp>


namespace FabricMaintenance {
    
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        PCL_ADD_NORMAL4D;
        PCL_ADD_RGB;
        float intensity;
        float curvature;
        uint32_t t;
        uint16_t reflectivity;
        uint16_t ring;
        uint16_t ambient;
        uint32_t range;
        float sensor_a;
        float sensor_b;
        float sensor_c;
        float lidar_label;
        float semantic_label;
        uint32_t collection_id;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace FabricMaintenance

POINT_CLOUD_REGISTER_POINT_STRUCT(FabricMaintenance::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (std::uint32_t, rgb, rgb)
    (float, intensity, intensity)
    (float, curvature, curvature)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
    (float, sensor_a, sensor_a)
    (float, sensor_b, sensor_b)
    (float, sensor_c, sensor_c)
    (float, lidar_label, lidar_label)
    (float, semantic_label, semantic_label)
    (std::uint32_t, collection_id, collection_id)
)

PCL_INSTANTIATE(NormalEstimationOMP, FabricMaintenance::Point);
PCL_INSTANTIATE(KdTree, FabricMaintenance::Point);
PCL_INSTANTIATE(transformPointCloud, FabricMaintenance::Point);
PCL_INSTANTIATE(IterativeClosestPoint, FabricMaintenance::Point);
PCL_INSTANTIATE(IterativeClosestPointNonLinear, FabricMaintenance::Point);
PCL_INSTANTIATE(PassThrough, FabricMaintenance::Point);
PCL_INSTANTIATE(ExtractIndices, FabricMaintenance::Point);
