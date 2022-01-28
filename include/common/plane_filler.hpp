#pragma once

#include <vector>
#include <cfloat>
#include <cmath>

#include <Eigen/Core>

#include <glog/logging.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "common/pcl_and_eigen_converter.hpp"

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;

class PlaneFiller
{
public:
    PlaneFiller() = default;
    ~PlaneFiller() = default;

    void initialize();

    bool fillPlane();

    bool preProcess();
    PointCloudPtr removeOutliers(PointCloud::ConstPtr input_cloud);
    PointCloudPtr downsampleCloud(PointCloud::ConstPtr input_cloud) const;

    bool getMinMaxBound();

    bool extractPlane();

    std::vector<Eigen::Vector3f> generatePointsOnPlane();

    PointCloudPtr get_filled_cloud() const { return filled_cloud_; };

    void set_input_cloud(PointCloudPtr input_cloud);
    void set_working_cloud(PointCloudPtr working_cloud);

    void set_plane_resolution(double plane_resolution);

    void set_downsample_voxel_size(double voxel_size);

    void set_plane_height(double plane_height);
    double get_plane_height() const { return plane_.height_; }


private:
    PointCloudPtr input_cloud_;
    PointCloudPtr working_cloud_;
    PointCloudPtr filled_cloud_;

    std::vector<Eigen::Vector3f> points_on_plane_;

    // TODO to make methods below static
    struct Plane
    {
        double resolution_ = 0.05;
        double height_ = 0.0;
        Eigen::Vector2f min_bound_ { FLT_MAX, FLT_MAX };
        Eigen::Vector2f max_bound_ { FLT_MIN, FLT_MIN };
        std::vector<Eigen::Vector3f> points_;
    } plane_;

    struct OutlierRemovalParameters
    {
        bool is_remove_outliers_ = false;
        int mean_k_ = 10;
        double stddev_threshold_ = 1.0;
    } outliers_removal_;

    struct DownsamplingParameters 
    {
        bool is_downsample_cloud_ = true;
        Eigen::Vector3d voxel_size_{0.01, 0.01, 0.01};
    } downsampling_;

    double grid_map_resolution_;

    struct PlaneExtratorPrameters
    {
        bool optimized_coefficients_ = true;
        double distance_threshold_ = 0.01;
        double max_iterations_ = 100;
    } plane_extrator_;
};
