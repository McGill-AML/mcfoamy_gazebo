#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_srvs/Trigger.h"

#include <gazebo/physics/physics.hh>
#include "boost/foreach.hpp"
#include "gazebo_example/Trajectory2.cpp"
#include "visualization_msgs/Marker.h"

namespace gazebo_example
{

class MotionplanNode
{
public:
  static const unsigned int MAX_PUB_QUEUE;
  static const unsigned int MAX_SUB_QUEUE;
  
  MotionplanNode();
  bool init();
  void run();
  
private:
  bool start_;
  
  ros::NodeHandle node_handle;
  ros::Publisher init_pose_pub_;
  //ros::Publisher init_twist_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher traj_pcl_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber points_sub_;
  ros::ServiceServer start_service_; 
  ros::Publisher vis_pub ;


  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;

  geometry_msgs::Pose init_pose_;
  std_msgs::Int16 trajectory_;
  pcl::PointCloud<pcl::PointXYZ> traj_pcl_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr traj_pcl_ptr_ = pcl::PointCloud<pcl::PointXYZ>::Ptr (&traj_pcl_);

  //pcl::PointCloud<pcl::PointXYZ>::Ptr traj_pcl_;

  gazebo::math::Vector3 p_0;
  gazebo::math::Vector3 p_final;
  gazebo::math::Vector3 p_global;
  gazebo::math::Quaternion q_global;

  double u_global;
  double theta_global;
  double psi_global;



  pcl::PointCloud<pcl::PointXYZ> points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (&points_);
  //pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> (.1);//resolution is 128
  std::vector<int> indices;
  //pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  std::vector<std::string> filenames;

  TrajectoryLibrary Traj_Lib;

  void wait_for_trigger();
  void compute_refstate();

  
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
  bool start_motionplan(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
};
  
}
