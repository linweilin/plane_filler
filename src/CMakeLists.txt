set(common_src common/plane_filler.cpp)

add_library(pcl_and_eigen_converter common/pcl_and_eigen_converter.cpp)
target_link_libraries(pcl_and_eigen_converter glog::glog ${PCL_LIBRARIES})

add_library(plane_filler SHARED ${common_src})
target_link_libraries(plane_filler pcl_and_eigen_converter glog::glog ${PCL_LIBRARIES})
