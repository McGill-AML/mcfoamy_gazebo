#include "gazebo_example/controller_node.h"


namespace gazebo_example
{
  
const unsigned int ControllerNode::MAX_PUB_QUEUE = 10;
const unsigned int ControllerNode::MAX_SUB_QUEUE = 10;
  
  
ControllerNode::ControllerNode():
  start_(false)
  , node_handle("")
  , pid_(0.0, 0.0, 0.0)
{}

bool ControllerNode::init()
{
  actuator_pub_ = node_handle.advertise<gazebo_example::actuator>("actuator", MAX_PUB_QUEUE);

  pose_sub_ = node_handle.subscribe("pose", MAX_SUB_QUEUE, 
                                    &ControllerNode::poseCallback, this);
  twist_sub_ = node_handle.subscribe("twist", MAX_SUB_QUEUE, 
                                     &ControllerNode::twistCallback, this);
  refpose_sub_ = node_handle.subscribe("refpose", MAX_SUB_QUEUE, 
                                    &ControllerNode::refposeCallback, this);
  reftwist_sub_ = node_handle.subscribe("reftwist", MAX_SUB_QUEUE, 
                                     &ControllerNode::reftwistCallback, this);
  start_service_ = node_handle.advertiseService("start_controller",
                                                &ControllerNode::start_controller,
                                                this);
  pid_ = controllers::PID(5.0, 5.0, 0.0);
  return true;
}

void ControllerNode::run()
{
  wait_for_trigger();
  
  const double frequency = 200.0; 
  ros::Rate loop_rate(frequency);
  
  while(ros::ok())
  {
    // Handle Callbacks
    ros::spinOnce();
    
    // Compute and publish controller output
    actuator_pub_.publish(compute_control_actuation(frequency));
    
    // Wait to maintain constant frequency
    loop_rate.sleep();
  }
}

void ControllerNode::wait_for_trigger()
{
  while(ros::ok())
  {
    ros::spinOnce();
    if(start_) {break;}
    ROS_INFO_THROTTLE(10, "Waiting for control start trigger");
  }
}

gazebo_example::actuator ControllerNode::compute_control_actuation(const double frequency)
{

  gazebo::math::Vector3 _r_a(pose_.position.x, -pose_.position.y, -pose_.position.z);
  gazebo::math::Quaternion _q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  gazebo::math::Vector3 _v_a(twist_.linear.x, -twist_.linear.y, -twist_.linear.z);
  gazebo::math::Vector3 _omega_a(twist_.angular.x, -twist_.angular.y, -twist_.angular.z);
  gazebo::math::Matrix3 _C_ba = _q.GetAsMatrix3();
  gazebo::math::Vector3 _v_b = _C_ba*_v_a;
  gazebo::math::Vector3 _omega_b = _C_ba*_omega_a;

  gazebo::math::Vector3 _r_ref_a(refpose_.position.x, refpose_.position.y, refpose_.position.z);
  gazebo::math::Quaternion _q_ref(refpose_.orientation.w, refpose_.orientation.x, refpose_.orientation.y, refpose_.orientation.z);
  gazebo::math::Vector3 _v_ref_r(reftwist_.linear.x, reftwist_.linear.y, reftwist_.linear.z);
  gazebo::math::Vector3 _omega_ra_r(reftwist_.angular.x, reftwist_.angular.y, reftwist_.angular.z);
  gazebo::math::Matrix3 _C_ra = _q_ref.GetAsMatrix3();

  double r_a[3] = {_r_a[0], _r_a[1], _r_a[2]};
  double C_ba[9] = {_C_ba[0][0],_C_ba[1][0],_C_ba[2][0],_C_ba[0][1],_C_ba[1][1],_C_ba[2][1],_C_ba[0][2],_C_ba[1][2],_C_ba[2][2]};
  double v_b[3] = {_v_b[0], _v_b[1], _v_b[2]};
  double omega_ba_b[3] = {_omega_b[0], _omega_b[1], _omega_b[2]};

  double r_ref_a[3] = {_r_a[0], _r_ref_a[1], _r_ref_a[2]};
  double C_ra[9] = {_C_ra[0][0],_C_ra[1][0],_C_ra[2][0],_C_ra[0][1],_C_ra[1][1],_C_ra[2][1],_C_ra[0][2],_C_ra[1][2],_C_ra[2][2]};
  double v_ref_r[3] = {_v_ref_r[0], _v_ref_r[1], _v_ref_r[2]};
  double omega_ra_r[3] = {_omega_ra_r[0], _omega_ra_r[1], _omega_ra_r[2]};

  double u1;
  double u2;
  double u3;
  double u4;

  controller_eb(C_ba, r_a, v_b, omega_ba_b, C_ra, r_ref_a, v_ref_r, omega_ra_r, 1.0 / frequency, maneuver_switch, &u1, &u2, &u3, &u4);


  gazebo_example::actuator command_actuator;
  command_actuator.u1 = u1; //aileron (Rad)
  command_actuator.u2 = u2; //elevator (Rad)
  command_actuator.u3 = u3; //rudder (Rad)
  command_actuator.u4 = u4; //throttle (rpm)
  
  return command_actuator;
}


void ControllerNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  pose_ = *msg;
}

void ControllerNode::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  twist_ = *msg;
}

void ControllerNode::refposeCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  refpose_ = *msg;
}

void ControllerNode::reftwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  reftwist_ = *msg;
}

bool gazebo_example::ControllerNode::start_controller(std_srvs::Trigger::Request& req,
                                                      std_srvs::Trigger::Response& res)
{
  start_ = true;
  res.success = true;
}

} // gazebo_example namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  
  gazebo_example::ControllerNode node;
  if(!node.init())
  {
    return 0;
  }
  
  node.run();

  return 0;
}
