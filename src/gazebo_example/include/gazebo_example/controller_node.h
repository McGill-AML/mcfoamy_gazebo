#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Trigger.h"

#include "gazebo_example/controller.h"

#include <gazebo/physics/physics.hh>
#include "gazebo_example/Trajectory3.cpp"
#include "gazebo_example/LowPassFilter2p.cpp"
#include "visualization_msgs/Marker.h"

#include <Eigen/Core>


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
  ros::Publisher ref_pose_pub_;
  ros::Publisher ref_twist_pub_;
  ros::Publisher ref_plane_vis_pub_ ;
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber trajectories_sub_;
  ros::ServiceServer start_service_;
  
  controllers::PID pid_;
  double maneuver_switch;
  int previous_trajectory;
  std::vector<double> trajectory_array_starttime;
  double trajectory_starttime;
  double trajectory_starttime_old;
  double trajectory_time;
  int trajectory;
  int current_trajectory;
  int trajectory_old;
  bool new_trajectory_recieved;
  bool new_trajectories_recieved;
  int trajectory_in_array;


  float delta_hi_i;

  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Pose init_pose_;
  //geometry_msgs::Twist init_twist_;
  std_msgs::Int16 trajectory_;
  std_msgs::Int16MultiArray trajectories_;
  geometry_msgs::Pose ref_pose_;
  geometry_msgs::Twist ref_twist_;
  std_msgs::Float64MultiArray command_actuator;
  gazebo::math::Vector3 initial_position;
  gazebo::math::Quaternion initial_quaternion;
  gazebo::math::Vector3 node_position;
  gazebo::math::Quaternion node_quaternion;
  std::vector<std::string> filenames;
  TrajectoryLibrary Traj_Lib;
  Eigen::VectorXd reference_state;
  LowPassFilter2p lp_Vs;
  double omega_t_old;
  void wait_for_trigger();
  double saturate(double value, double min, double max);

  gazebo::math::Vector3 GetReferencePosition(Eigen::VectorXd state, gazebo::math::Vector3 initial_position, double initial_yaw);
  gazebo::math::Quaternion GetReferenceQuaternion(Eigen::VectorXd state, double initial_yaw);
  std_msgs::Float64MultiArray compute_control_actuation(const double frequency);
  
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void init_poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void trajectoryCallback(const std_msgs::Int16::ConstPtr& msg);
  void trajectoriesCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);

  bool start_controller(std_srvs::Trigger::Request  &req,
                        std_srvs::Trigger::Response &res);
};
  
}
