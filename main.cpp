
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <unistd.h>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

// #include <ros/ros.h>
// #include <ros/package.h>
#include "rclcpp/rclcpp.hpp"
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


#include "read_configs.h"
#include "dataset.h"
#include "camera.h"
#include "frame.h"
#include "map_stitcher.h"
#include "map_builder.h"
#include "thread_publisher.h"
// #include "visualization.h"

#include <typeinfo>
#include <sys/io.h>
#include <sys/dir.h>
#include <time.h>

using namespace std;

struct MyTime{
  // seconds
  std::chrono::seconds sec;
  // nanoseconds
  std::chrono::nanoseconds nsec;
};

struct MyHeader{
  // frame id
  std::string frame_id;
  // timestamp
  MyTime stamp;
};

struct MyPosition{
  // x var
  float x = 0.0;
  // y var
  float y = 0.0;
  // z var
  float z = 0.0;

};

struct MyOrientation{
  // x var
  float x = 0.0;
  // y var
  float y = 0.0;
  // z var
  float z = 0.0;
  // w var
  float w = 0.0;
};

struct MyPose{
  // position
  MyPosition position;
  // orientation
  MyOrientation orientation;
};

struct MyPoseStamped{
  // Header
  MyHeader header;
  // Pose
  MyPose pose;
};

struct MyPath{
  // Header
  MyHeader header;
  // poses
  std::vector<MyPoseStamped> poses;
};

class Visualizer{
public:
  enum class TrajectoryType {
    Frame = 0,
    KCC = 1,
    Odom = 2,
  };

  Visualizer(VisualizationConfig& config);
  void AddNewPoseToPath(
    Eigen::Vector3d& pose, double time_double, MyPath& path, std::string& frame_id);
  // void UpdateOdomPose(Eigen::Vector3d& pose, double time_double);
  void UpdateKccPose(Eigen::Vector3d& pose, double time_double);
  void UpdateFramePose(Aligned<std::vector, Eigen::Vector3d>& frame_poses, std::vector<double>& timestamps);
  // void ConvertMapToOccupancyMsgs(OccupancyData& map, nav_msgs::msg::OccupancyGrid& msgs);
  // void UpdateMap(MapBuilder& map_builder);

  // void PublishImage(cv::Mat& image, double time_double);
  void GetTrajectoryTxt(std::vector<std::vector<std::string> >& lines, TrajectoryType trajectory_type);

private:
  std::string frame_id;

  MyPath odom_pose_msgs;
  MyPath kcc_pose_msgs;
  MyPath frame_pose_msgs;
  // nav_msgs::msg::OccupancyGrid occupancy_map_msgs;
};

MyTime getCurrentTime();
MyOrientation euler_to_quaternion(float yaw, float pitch, float roll);

int main(int argc, char** argv){
  // rclcpp::init(argc, argv, "build_map");
  // rclcpp::start(); 

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config; 
  Dataset dataset(dataset_config.dataroot, dataset_config.image_dir_name); 

  MapBuilder map_builder(configs); 
  Visualizer visualizer(configs.visualization_config); 
  Aligned<std::vector, Eigen::Vector3d> frame_poses;
  std::vector<double> timestamps;
  Eigen::Vector3d new_kcc_pose; 
  // TODO: Find way to loop at rate
  // rclcpp::Rate loop_rate(50); 
  int loop_rate = 50;
  int period_ms = 1000 / loop_rate;
  std::vector<std::vector<std::string> > frame_lines;
  size_t dataset_length = dataset.GetDatasetLength();

  for(size_t i = 0; i < dataset_length; ++i){ 
    // if(!ros::ok()) break; 
    std::cout << i << std::endl;
    cv::Mat image;
    if(!dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }
    double time_double = dataset.GetTimestamp(i);
    // visualizer.PublishImage(image, time_double);
    auto t1 = std::chrono::high_resolution_clock::now();
    bool insert_keyframe = map_builder.AddNewInput(image, time_double); // error if image size is wrong
    auto t2 = std::chrono::high_resolution_clock::now();
    auto compute_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1e3;
    cout << "processing for one frame is " << compute_time << "ms" << endl;

    std::cout << "Lopp" << endl;
    if((i + 1) >= dataset_length){
      map_builder.CheckAndOptimize();
    }else if(!insert_keyframe){
      continue;
    };  
    std::cout << "Insert a keyframe !" << std::endl;

    // publish pose msgs
    if(map_builder.GetCFPose(new_kcc_pose)){    
      visualizer.UpdateKccPose(new_kcc_pose, time_double);
      std::cout << new_kcc_pose << std::endl;
    }
    if(map_builder.GetFramePoses(frame_poses, timestamps)){
      // std::cout << timestamps << std::endl;
      visualizer.UpdateFramePose(frame_poses, timestamps);
    }

    std::cout << "Skipped if statements" << std::endl;
    // visualizer.UpdateMap(map_builder);
    // ros::spinOnce(); 
    // loop_rate.sleep(); 
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  // save trajectories 
  std::cout << "Saving data..." << std::endl;
  std::string saving_root = configs.saving_config.saving_root;
  MakeDir(saving_root);
  std::string trajectory_KCC = saving_root + "/KCC_Keyframe.txt";
  std::string trajectory_frame = saving_root + "/optimized_keyframe.txt";
  std::vector<std::vector<std::string> > kcc_keyframe_lines, optimized_keyframe_lines;
  visualizer.GetTrajectoryTxt(kcc_keyframe_lines, Visualizer::TrajectoryType::KCC);
  visualizer.GetTrajectoryTxt(optimized_keyframe_lines, Visualizer::TrajectoryType::Frame);

  WriteTxt(trajectory_KCC, kcc_keyframe_lines, " ");
  WriteTxt(trajectory_frame, optimized_keyframe_lines, " ");
};

Visualizer::Visualizer(VisualizationConfig& config): frame_id(config.frame_id){
  // odom_pose_pub = nh.advertise<nav_msgs::Path>(config.odom_pose_topic, 10);
  // kcc_pose_pub = nh.advertise<nav_msgs::msg::Path>(config.kcc_pose_topic, 10);
  // frame_pose_pub = nh.advertise<nav_msgs::msg::Path>(config.frame_pose_topic, 10);
  // map_pub = nh.advertise<nav_msgs::msg::OccupancyGrid>(config.map_topic, 1);
  // image_pub = nh.advertise<sensor_msgs::msg::Image>(config.image_topic, 1);

  // rclcpp::Time current_time = rclcpp::Time::now();
  MyTime current_time = getCurrentTime();
  odom_pose_msgs.header.stamp = current_time; 
	odom_pose_msgs.header.frame_id = frame_id; 
  kcc_pose_msgs.header.stamp = current_time; 
	kcc_pose_msgs.header.frame_id = frame_id; 
  frame_pose_msgs.header.stamp = current_time; 
	frame_pose_msgs.header.frame_id = frame_id; 

//   occupancy_map_msgs.header.stamp = current_time;
//   occupancy_map_msgs.header.frame_id = frame_id;
}

void Visualizer::GetTrajectoryTxt(
    std::vector<std::vector<std::string> >& lines, TrajectoryType trajectory_type){
  MyPath* path_ptr;
  switch(trajectory_type){
    case TrajectoryType::Frame:
      path_ptr = &frame_pose_msgs;
      break;
    case TrajectoryType::KCC:
      path_ptr = &kcc_pose_msgs;
      break;
    case TrajectoryType::Odom:
      path_ptr = &odom_pose_msgs;
      break;
    default:
      std::cout << "please select trajectory_type from Frame, KCC and Odom !" << std::endl; 
      return;
  }

  for(MyPoseStamped& pose_stamped : (*path_ptr).poses){
    std::vector<std::string> line;
    int64_t sec = pose_stamped.header.stamp.sec.count();
    int64_t nsec = pose_stamped.header.stamp.nsec.count();
    // std::string s_time = std::to_string(sec) + "." + std::to_string(nsec);

    double time_double = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
    std::string s_time = std::to_string(time_double);

    line.emplace_back(s_time);
    line.emplace_back(std::to_string(pose_stamped.pose.position.x));
    line.emplace_back(std::to_string(pose_stamped.pose.position.y));
    line.emplace_back(std::to_string(pose_stamped.pose.position.z));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.x));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.y));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.z));
    line.emplace_back(std::to_string(pose_stamped.pose.orientation.w));
    lines.push_back(line);
  }
}

void Visualizer::UpdateKccPose(Eigen::Vector3d& pose, double time_double){
  AddNewPoseToPath(pose, time_double, kcc_pose_msgs, frame_id);
  // kcc_pose_pub.publish(kcc_pose_msgs); 
}

void Visualizer::UpdateFramePose(Aligned<std::vector, Eigen::Vector3d>& frame_poses, std::vector<double>& timestamps){
  frame_pose_msgs.poses.clear();
  for(size_t i = 0; i < frame_poses.size(); i++){
    AddNewPoseToPath(frame_poses[i], timestamps[i], frame_pose_msgs, frame_id);
  }
  // frame_pose_pub.publish(frame_pose_msgs); 

}

void Visualizer::AddNewPoseToPath(
    Eigen::Vector3d& pose, double time_double, MyPath& path, std::string& id){
  MyTime current_time = getCurrentTime();

  MyPoseStamped pose_stamped; 
  pose_stamped.pose.position.x = pose(0); 
  pose_stamped.pose.position.y = pose(1); 
  pose_stamped.pose.position.z = 0; 

  MyOrientation q = euler_to_quaternion(pose(2), 0.0, 0.0); 
  pose_stamped.pose.orientation.x = q.x; 
  pose_stamped.pose.orientation.y = q.y; 
  pose_stamped.pose.orientation.z = q.z; 
  pose_stamped.pose.orientation.w = q.w; 

  if(time_double < 0){
    pose_stamped.header.stamp = current_time; 
  }else{
    int64_t sec = static_cast<int64_t>(time_double);
    int64_t nsec = static_cast<int64_t>((time_double - sec) * 1e9);
    pose_stamped.header.stamp.nsec = std::chrono::nanoseconds(nsec);
    pose_stamped.header.stamp.sec = std::chrono::seconds(sec);
    std::string s_time = std::to_string(sec) + "." + std::to_string(nsec);
  }
  
  pose_stamped.header.frame_id = id; 
  path.poses.push_back(pose_stamped); 
}

MyTime getCurrentTime() {
    // Get the current time point using std::chrono::system_clock
    auto now = std::chrono::system_clock::now();

    // Extract seconds and nanoseconds components from the current time point
    auto since_epoch = now.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch - sec);

    // Create a MyTime object and store the components
    MyTime currentTime;
    currentTime.sec = sec;
    currentTime.nsec = nsec;

    return currentTime;
}

MyOrientation euler_to_quaternion(float yaw, float pitch, float roll){
  MyOrientation q;
  q.x = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - \
      std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
  q.y = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + \
      std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
  q.z = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - \
      std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
  q.w = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + \
      std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
  return q;
}