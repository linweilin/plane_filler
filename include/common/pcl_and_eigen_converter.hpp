#pragma once

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <glog/logging.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(const std::vector<Eigen::Vector3f> source_cloud);
