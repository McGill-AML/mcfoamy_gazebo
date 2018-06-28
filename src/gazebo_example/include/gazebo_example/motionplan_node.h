#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"

#include "gazebo_example/actuator.h"

#include <gazebo/physics/physics.hh>

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "boost/foreach.hpp"
#include "maneuver_generator/maneuver_generator.h"

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
  ros::Publisher refpose_pub_;
  ros::Publisher reftwist_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber points_sub_;
  ros::ServiceServer start_service_; 

  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;

  geometry_msgs::Pose refpose_;
  geometry_msgs::Twist reftwist_;

  gazebo::math::Vector3 p_0;
  gazebo::math::Vector3 p_final;
  gazebo::math::Vector3 p_global;
  gazebo::math::Quaternion q_global;

  double u_global;
  double theta_global;
  double psi_global;



  pcl::PointCloud<pcl::PointXYZ> points_;
  
  void wait_for_trigger();
  void compute_refstate();

  
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
  bool start_motionplan(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
};
  
}
