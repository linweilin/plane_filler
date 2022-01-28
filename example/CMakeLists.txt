add_executable(test_plane_filler test_plane_filler.cpp)
target_link_libraries(test_plane_filler plane_filler glog::glog ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
