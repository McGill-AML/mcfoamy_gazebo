
#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"

#include "gazebo_example/controller.h"

namespace gazebo_example
{

class ControllerNode
{
public:
  static const unsigned int MAX_PUB_QUEUE;
  static const unsigned int MAX_SUB_QUEUE;
  
   ControllerNode();
  bool init();
  void run();
  
private:
  bool start_;
  
  ros::NodeHandle node_handle;
  ros::Publisher wrench_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::ServiceServer start_service_;
  
  controllers::PID pid_;
  
  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  
  void wait_for_trigger();
  geometry_msgs::Wrench compute_control_wrench(const double frequency);
  
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  bool start_controller(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
};
  
}
