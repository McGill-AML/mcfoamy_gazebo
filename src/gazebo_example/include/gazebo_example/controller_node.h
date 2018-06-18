#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Trigger.h"

#include "gazebo_example/controller.h"
#include "gazebo_example/actuator.h"
#include "controller_eb/controller_eb.h"
#include "maneuver_generator/maneuver_generator.h"

#include <gazebo/physics/physics.hh>


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
  ros::Publisher actuator_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::ServiceServer start_service_;
  
  controllers::PID pid_;
    double maneuver_switch;

  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  
  void wait_for_trigger();
  gazebo_example::actuator compute_control_actuation(const double frequency);
  
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  bool start_controller(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
};
  
}
