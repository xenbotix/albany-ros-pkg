cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Enable dynamic_reconfigure
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

# ---[ Point Cloud Library - Segmentation
rosbuild_add_library (piece_extraction
                      src/extract_pieces.cpp
                     )
#rosbuild_add_compile_flags (pcl_ros_segmentation ${SSE_FLAGS})
#target_link_libraries (pcl_ros_segmentation pcl_segmentation pcl_ros_tf)
rosbuild_link_boost (piece_extraction system filesystem)
