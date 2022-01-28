#include <iostream>
#include "common/plane_filler.hpp"
#include "common/pcl_and_eigen_converter.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();

    std::cout << "Hello, this is plane_filler" << std::endl;

    std::string input_pcd_file("data/combined_groud.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *input_cloud) == -1) {

        LOG(FATAL) << "Couldn't read input pcd file";
        return(-1);
    }
    
    auto plane_filler = std::make_shared<PlaneFiller>();
    plane_filler->initialize();
    plane_filler->set_input_cloud(input_cloud);
    // TODO get resolution as downsample voxel size
    plane_filler->set_downsample_voxel_size(0.05);
    if(plane_filler->fillPlane() == true) {

        auto result_cloud = plane_filler->get_filled_cloud();
        LOG(INFO) << "Generate filled plane cloud successfully.";

        // pcl::visualization::CloudViewer viewer("Cloud Viewer");
        // viewer.showCloud(result_cloud);
        // while (!viewer.wasStopped())
        // {

        // }

        pcl::io::savePCDFile("/home/william/Downloads/filled_cloud.pcd", *result_cloud);
    }
    else
        LOG(INFO) << "Failed to generate filled plane cloud. Something wrong?";

    return 0;
}
