#pragma once

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "gazebo_example/actuator.h"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>


#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "McFoamy_FM_v2/McFoamy_FM_v2.h"


namespace gazebo
{

class PlanePlugin : public ModelPlugin
{
public: 
  PlanePlugin();
  virtual ~PlanePlugin();
    
  // Load the controller
  void Load( physics::ModelPtr parent, sdf::ElementPtr sdf );

protected:
  // Gazebo related attributes
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  
  // Mutex for the update
  boost::mutex lock_;
  
  // Force/Torque for the update
  math::Vector3 force_;
  math::Vector3 torque_;

  double actuator_[4];
  // ROS related attributes
  uint8_t MAX_PUB_QUEUE_SIZE;
  uint8_t MAX_SUB_QUEUE_SIZE;
  
  std::string robot_namespace_;
  event::ConnectionPtr update_connection_;
  boost::shared_ptr<ros::NodeHandle> rosnode_;
  
  // ROS Subscribers/Publishers
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  ros::Subscriber actuator_sub_;

  
  // ROS Subscribers callbacks
  void actuatorCallback(const gazebo_example::actuator::ConstPtr& actuator_msg);

  
  // Custom Callback Queue for ROS messages
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();
  
  // Initialization functions
  void ParseParameters(sdf::ElementPtr sdf);
  void InitROSNode();
  
  // Update the controller
  virtual void UpdateChild();
  
  void publishLinkState();
  double saturate(double value, double min, double max);
  void aerodynamics();

  double Fx_b;
  double Fy_b;
  double Fz_b;
  double Mx_b;
  double My_b;
  double Mz_b;

};


}
