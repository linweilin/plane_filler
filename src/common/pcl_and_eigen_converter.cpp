#include "common/pcl_and_eigen_converter.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud(const std::vector<Eigen::Vector3f> source_cloud)
{
    if(!source_cloud.empty()) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ temp_point;
        for(auto &p : source_cloud) {

            temp_point.getVector3fMap() = p;
            cloud->points.push_back(temp_point);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;

        return cloud;
    }
    else {
        LOG(WARNING) << "Failed to convert Eigen vector to point cloud. Input source Eigen vector is empty.";
        return nullptr;
    }
}
