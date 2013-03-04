cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set (MESH_DIR ${PROJECT_SOURCE_DIR})

install(DIRECTORY meshes
    DESTINATION ${MESH_DIR}
    FILES_MATCHING PATTERN "*.STL"
    PATTERN ".git" EXCLUDE)

