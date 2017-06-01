cmake_minimum_required(VERSION 2.8.3)

project(image_undistort)

# Set compile flags (-Ofast actually makes a big difference over -O3 here (maybe 20% faster)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
  tf_conversions
  nodelet
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_nodelet 
  src/image_undistort_nodelet.cpp 
  src/stereo_info_nodelet.cpp
  src/stereo_undistort_nodelet.cpp
  src/stereo_info.cpp
  src/stereo_undistort.cpp
  src/depth.cpp
  src/depth_nodelet.cpp
  src/camera_parameters.cpp 
  src/image_undistort.cpp 
  src/undistorter.cpp)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet ${catkin_EXPORTED_TARGETS})

add_executable(image_undistort_node src/image_undistort_node.cpp)
target_link_libraries(image_undistort_node
  ${catkin_LIBRARIES}
)

add_executable(stereo_info_node src/stereo_info_node.cpp)
target_link_libraries(stereo_info_node
  ${catkin_LIBRARIES}
)

add_executable(stereo_undistort_node src/stereo_undistort_node.cpp)
target_link_libraries(stereo_undistort_node
  ${catkin_LIBRARIES}
)

add_executable(depth_node src/depth_node.cpp)
target_link_libraries(depth_node
  ${catkin_LIBRARIES}
)

add_executable(dense_stereo_node src/dense_stereo_node.cpp)
target_link_libraries(dense_stereo_node
  ${catkin_LIBRARIES}
)

# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# Install xml files
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
