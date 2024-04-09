#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <Eigen/Core>

// #include <ros/ros.h>
// #include <ros/package.h>
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include <visualization_msgs/Marker.h>
#include "visualization_msgs/msg/marker.hpp"
// #include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
// #include <nav_msgs/Odometry.h>
#include "nav_msgs/msg/odometry.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "read_configs.h"
#include "map_stitcher.h"
#include "map_builder.h"

class Visualizer{
public:
  enum class TrajectoryType {
    Frame = 0,
    KCC = 1,
    Odom = 2,
  };

  Visualizer(VisualizationConfig& config);
  void AddNewPoseToPath(
    Eigen::Vector3d& pose, double time_double, nav_msgs::msg::Path& path, std::string& frame_id);
  void UpdateOdomPose(Eigen::Vector3d& pose, double time_double);
  void UpdateKccPose(Eigen::Vector3d& pose, double time_double);
  void UpdateFramePose(Aligned<std::vector, Eigen::Vector3d>& frame_poses, std::vector<double>& timestamps);
  void ConvertMapToOccupancyMsgs(OccupancyData& map, nav_msgs::msg::OccupancyGrid& msgs);
  void UpdateMap(MapBuilder& map_builder);

  void PublishImage(cv::Mat& image, double time_double);
  void GetTrajectoryTxt(std::vector<std::vector<std::string> >& lines, TrajectoryType trajectory_type);

private:
  rclcpp::NodeHandle nh;
  std::string frame_id;
  rclcpp::Publisher odom_pose_pub;
  rclcpp::Publisher kcc_pose_pub;
  rclcpp::Publisher frame_pose_pub;
  rclcpp::Publisher map_pub;
  rclcpp::Publisher image_pub;

  nav_msgs::msg::Path odom_pose_msgs;
  nav_msgs::msg::Path kcc_pose_msgs;
  nav_msgs::msg::Path frame_pose_msgs;
  nav_msgs::msg::OccupancyGrid occupancy_map_msgs;
};

#endif  // VISUALIZATION_H_