#include "common/plane_filler.hpp"

void PlaneFiller::initialize()
{
    LOG(INFO) << "PlaneFiller initializing...";
    PointCloudPtr initilized_input_cloud(new PointCloud);
    PointCloudPtr initilized_working_cloud(new PointCloud);
    PointCloudPtr initilized_filled_cloud(new PointCloud);

    input_cloud_ = initilized_input_cloud;
    working_cloud_ = initilized_working_cloud;
    filled_cloud_ = initilized_filled_cloud;
    LOG(INFO) << "PlaneFiller initialized.";
}

bool PlaneFiller::fillPlane()
{
    LOG(INFO) << "Start to fill plane...";
    preProcess();

    getMinMaxBound();

    extractPlane();

    points_on_plane_ = generatePointsOnPlane();

    filled_cloud_ = convertToPointCloud(points_on_plane_);

    LOG(INFO) << "Plane is filled.";
    return true;
}

PointCloudPtr PlaneFiller::removeOutliers(PointCloud::ConstPtr input_cloud)
{
    LOG(INFO) << "Removing outliers...";
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(outliers_removal_.mean_k_);
    sor.setStddevMulThresh(outliers_removal_.stddev_threshold_);
    PointCloudPtr filtered_cloud(new PointCloud());
    sor.filter(*filtered_cloud);
    LOG(INFO) << "Removed outliers.";
    return filtered_cloud;
}

PointCloudPtr PlaneFiller::downsampleCloud(PointCloud::ConstPtr input_cloud) const 
{
    LOG(INFO) << "Dowsampling cloud...";
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(input_cloud);
    const auto& voxel_size = downsampling_.voxel_size_;
    voxelGrid.setLeafSize(voxel_size.x(), voxel_size.y(), voxel_size.z());
    PointCloudPtr downsampled_cloud(new PointCloud());
    voxelGrid.filter(*downsampled_cloud);
    LOG(INFO) << "Dowsampled cloud.";
    return downsampled_cloud;
}

bool PlaneFiller::preProcess()
{
    // Preprocess: Remove outliers, downsample cloud, transform cloud
    LOG(INFO) << "Preprocessing of the pointcloud started";

    if (outliers_removal_.is_remove_outliers_) {

        auto filtered_cloud = removeOutliers(working_cloud_);
        set_working_cloud(filtered_cloud);
    }

    if (downsampling_.is_downsample_cloud_) {

        auto downsampled_cloud = downsampleCloud(working_cloud_);
        set_working_cloud(downsampled_cloud);
    }

    if(working_cloud_->is_dense == true) {

        working_cloud_->is_dense = false;
        LOG(INFO) << "Set cloud is_dense = false.";
    }

    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*working_cloud_, *working_cloud_, mapping); 
    LOG(INFO) << "Remove NaN from cloud.";

    LOG(INFO) << "Preprocessing and filtering finished";
    return true;
}

bool PlaneFiller::getMinMaxBound()
{
    LOG(INFO) << "Start to get min and max bound of plane...";

    PointType min_bound;
    PointType max_bound;
    pcl::getMinMax3D(*working_cloud_, min_bound, max_bound);
    
    plane_.min_bound_(0) = std::floor(min_bound.x);
    plane_.min_bound_(1) = std::floor(min_bound.y);
    plane_.max_bound_(0) = std::ceil(max_bound.x);
    plane_.max_bound_(1) = std::ceil(max_bound.y);

    LOG(INFO) << "Got min and max bound of plane.";

    return true;
}

bool PlaneFiller::extractPlane()
{
    
    LOG(INFO) << "Start to extract plane...";

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (plane_extrator_.optimized_coefficients_);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (plane_extrator_.distance_threshold_);
    seg.setMaxIterations(plane_extrator_.max_iterations_);
    seg.setInputCloud (working_cloud_);
    seg.segment (*inliers, *coefficients);
    LOG(INFO) << "plane inliers indices size = " << inliers->indices.size () << std::endl;

    LOG(INFO) << "Plane extracted.";

    plane_.height_ = working_cloud_->points.at(inliers->indices.at(0)).z;
    LOG(INFO) << "Got plane heigh = " << plane_.height_;

    return true;
}

std::vector<Eigen::Vector3f> PlaneFiller::generatePointsOnPlane()
{
    LOG(INFO) << "generating points on plane...";
    // plane_.resolution_ = downsampling_.voxel_size_(0) / 2.0;        // Make sure to have enough points in each voxel
    plane_.resolution_ = 0.06;        // Make sure to have enough points in each voxel
    auto plane_row = (plane_.max_bound_(1) - plane_.min_bound_(1)) / plane_.resolution_ + 1;
    auto plane_column = (plane_.max_bound_(0) - plane_.min_bound_(0)) / plane_.resolution_ + 1;
    std::vector<Eigen::Vector3f> temp_points (plane_row * plane_column, Eigen::Vector3f(0., 0., plane_.height_));
    for(decltype(temp_points.size()) row = 0; row < plane_row; ++row) {
        
        for(decltype(temp_points.size()) col = 0; col < plane_column; ++col) {
                
            auto point_column  = plane_.min_bound_(0) + plane_.resolution_ * col;
            auto point_row  = plane_.min_bound_(1) + plane_.resolution_ * row;
            temp_points.push_back(Eigen::Vector3f(point_column, point_row, plane_.height_));
        }
    }

    LOG(INFO) << "Points on plane generated, size = " << temp_points.size();
    return temp_points;
}

void PlaneFiller::set_input_cloud(PointCloudPtr input_cloud)
{
    if(input_cloud != nullptr) {

        if(!input_cloud->points.empty()) {
            
            input_cloud_ = input_cloud;
            *working_cloud_ = *input_cloud_;
            LOG(INFO) << "Input cloud points: " << working_cloud_->points.size();
        }
        else
            LOG(FATAL) << "Input cloud does not have any point.";
    }
    else
        LOG(FATAL) << "Input cloud is null!";
}

void PlaneFiller::set_working_cloud(PointCloudPtr working_cloud)
{
    working_cloud_ = working_cloud;
}

void PlaneFiller::set_downsample_voxel_size(double voxel_size)
{
    downsampling_.voxel_size_ = Eigen::Vector3d(voxel_size, voxel_size, voxel_size);
}

void PlaneFiller::set_plane_height(double plane_height)
{
    plane_.height_ = plane_height;
}

