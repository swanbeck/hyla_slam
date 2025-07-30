#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/instantiate.hpp>


namespace hylacomylus {
    
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
} // namespace hylacomylus

POINT_CLOUD_REGISTER_POINT_STRUCT(hylacomylus::Point,
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

PCL_INSTANTIATE(NormalEstimationOMP, hylacomylus::Point);
PCL_INSTANTIATE(KdTree, hylacomylus::Point);
PCL_INSTANTIATE(transformPointCloud, hylacomylus::Point);
PCL_INSTANTIATE(IterativeClosestPoint, hylacomylus::Point);
PCL_INSTANTIATE(IterativeClosestPointNonLinear, hylacomylus::Point);
PCL_INSTANTIATE(PassThrough, hylacomylus::Point);
PCL_INSTANTIATE(ExtractIndices, hylacomylus::Point);
PCL_INSTANTIATE(VoxelGrid, hylacomylus::Point);
