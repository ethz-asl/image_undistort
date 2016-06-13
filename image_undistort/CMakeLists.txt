cmake_minimum_required(VERSION 2.8.3)
project(image_undistort)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
 src/image_undistort.cpp
 src/undistorter.cpp
 src/cam_info_reader.cpp
 )

############
# BINARIES #
############
cs_add_executable(cam_info_reader_node
  src/cam_info_reader_node.cpp
)
cs_add_executable(image_undistort_node
  src/image_undistort_node.cpp
)

target_link_libraries(image_undistort_node ${PROJECT_NAME})
target_link_libraries(cam_info_reader_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
