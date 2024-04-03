cmake_minimum_required(VERSION 3.5.1)
project(ni_slam)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -Wno-reorder")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

##### REPLACING WITH AMENT VERSION #######
# find_package(catkin REQUIRED COMPONENTS
#   cv_bridge
#   # eigen_conversions
#   geometry_msgs
#   image_transport
#   nav_msgs
#   roscpp
#   rospy
#   std_msgs
#   sensor_msgs
# )
find_package(ament_cmake REQUIRED)
find_package(cv_bridge)
# find_package(geometry_msgs)
# find_package(visualization_msgs)
find_package(image_transport)
# find_package(nav_msgs REQUIRED)
# find_package(rclcpp)
# find_package(rclpy)
# find_package(std_msgs)
# find_package(sensor_msgs)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FFTW3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(VTK REQUIRED)

##### REPLACING WITH AMENT VERSION #######
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES correlation_flow
#  CATKIN_DEPENDS geometry_msgs visualization_msgs image_transport nav_msgs roscpp rospy std_msgs
# #  DEPENDS system_lib
# )

# install(DIRECTORY include/ ############### TODO: CHECK IF NECESSARY!!
#   DESTINATION include)
# from CATKIN_DEPENDS
# ament_export_dependencies(geometry_msgs visualization_msgs image_transport nav_msgs rclcpp rclpy std_msgs)
ament_export_dependencies(image_transport)
# from catkin INCLUDE_DIRS
ament_export_include_directories(include)
# from LIBRARIES
ament_export_libraries(correlation_flow)

###########
## Build ##
###########

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)
# ament_export_include_directories(
#   ${PROJECT_SOURCE_DIR}/include 
#   OpenCV
#   Eigen3
#   nav_msgs)

add_library(${PROJECT_NAME}_lib SHARED
  src/optimization_2d/pose_graph_2d.cc
  src/utils.cc
  src/dataset.cc
  src/camera.cc
  src/frame.cc
  src/edge.cc
  src/correlation_flow.cc
  src/loop_closure.cc
  src/map.cc
  src/map_builder.cc
  src/thread_publisher.cc
  src/map_stitcher.cc
  # src/visualization.cc
  src/timer.cc
)

target_link_libraries(${PROJECT_NAME}_lib
  -lfftw3 
  -lfftw3f
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${CERES_LIBRARIES}
  yaml-cpp
  ${Boost_LIBRARIES}
  # ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  # nav_msgs
  # geometry_msgs
  # visualization_msgs
  # image_transport
  # nav_msgs
  # std_msgs
)
ament_export_libraries(correlation_flow)

## Declare a C++ executable
add_executable(${PROJECT_NAME} main.cpp)
# ament_target_dependencies(${PROJECT_NAME} geometry_msgs visualization_msgs image_transport nav_msgs rclcpp rclpy std_msgs)
ament_target_dependencies(${PROJECT_NAME} image_transport)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_package()