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
  //const controllers::State desired_state = {5.0, 0};
  //const controllers::State measured_state = {pose_.position.z, twist_.linear.z};
  //math::Matrix3 C_ba = pose_.orientation.GetAsMatrix3();
  //const double force_z = pid_.output(desired_state, measured_state, 1.0 / frequency);
  //math::Vector3 omega_ba_a =  twist_.angular;
  gazebo::math::Vector3 _r_a(pose_.position.x, -pose_.position.y, -pose_.position.z);
  gazebo::math::Quaternion _q_g(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  gazebo::math::Quaternion _q = _q_g;

  gazebo::math::Vector3 _v_a(twist_.linear.x, -twist_.linear.y, -twist_.linear.z);
  gazebo::math::Vector3 _omega_a(twist_.angular.x, -twist_.angular.y, -twist_.angular.z);

  gazebo::math::Matrix3 _C_ba = _q.GetAsMatrix3();

  gazebo::math::Vector3 _v_b = _C_ba*_v_a;
  gazebo::math::Vector3 _omega_b = _C_ba*_omega_a;

  //gazebo::math::Vector3 _v_b = _q.GetInverse().GetAsMatrix3()*_v_a;
  //gazebo::math::Vector3 _omega_b =_q.GetInverse().GetAsMatrix3()*_omega_a;
  //gazebo::math::Quaternion _q_r(0.0,-3.14/2.0,0.0);
  //gazebo::math::Matrix3 _C_ra = _q_r.GetAsMatrix3();

  double maneuver_type = 1.0;
  double absolute_time = 0.0;
  double qref[4];
  double pref[3];
  double uref;
  double distance;





  double C_ba[9] = {_C_ba[0][0],_C_ba[1][0],_C_ba[2][0],_C_ba[0][1],_C_ba[1][1],_C_ba[2][1],_C_ba[0][2],_C_ba[1][2],_C_ba[2][2]};
  double r_a[3] = {_r_a[0], _r_a[1], _r_a[2]};
  //double r_a2[3] = {_r_a[0], _r_a[1], _r_a[2]};

  double v_b[3] = {_v_b[0], _v_b[1], _v_b[2]};
  double omega_ba_b[3] = {_omega_b[0], _omega_b[1], _omega_b[2]};
  //double C_ra[9] = {_C_ra[0][0],_C_ra[1][0],_C_ra[2][0],_C_ra[0][1],_C_ra[1][1],_C_ra[2][1],_C_ra[0][2],_C_ra[1][2],_C_ra[2][2]};
  //double r_ref_a[3] = {r_a[0], 0.0, -10.0};
  //double v_ref_r[3] = {5.0, 0.0, 0.0};
  double omega_ra_r[3] = {0.0, 0.0, 0.0};
  double u1;
  double u2;
  double u3;
  double u4;


  double q[4] = {_q.w, _q.x, _q.y, _q.z};
 // double q2[4] = {_q.w, _q.x, _q.y, _q.z};


  //maneuver_generator(maneuver_type, absolute_time, r_a, q, &maneuver_switch, &qref[4], &pref[3], &uref, &distance);
  double v_ref_r[3] = {0.0, 0.0, 0.0};
  gazebo::math::Quaternion _q_r(-.707,0.0,.707,0.0);

  //gazebo::math::Quaternion _q_r(qref[0], qref[1], qref[2], qref[3]);
  //gazebo::math::Quaternion _q_r(3.14/2.0,-3.14/5.0,0.0);

  gazebo::math::Matrix3 _C_ra = _q_r.GetAsMatrix3();
  double C_ra[9] = {_C_ra[0][0],_C_ra[1][0],_C_ra[2][0],_C_ra[0][1],_C_ra[1][1],_C_ra[2][1],_C_ra[0][2],_C_ra[1][2],_C_ra[2][2]};
  //double r_ref_a[3] = {pref[0], pref[1], -10.0};
double r_ref_a[3] = {0.0, 0.0, -10.0};
  controller_eb(C_ba, r_a, v_b, omega_ba_b, C_ra, r_ref_a, v_ref_r, omega_ra_r, 1.0 / frequency, maneuver_switch, &u1, &u2, &u3, &u4);


  gazebo_example::actuator command_actuator;
  //command_actuator.u3 = force_z;
  //command_actuator.u1 = 0.0; //aileron (Rad)
  //command_actuator.u2 = -.270; //elevator (Rad)
  //command_actuator.u3 = 0.0; //rudder (Rad)
  //command_actuator.u4 = 4000.0; //throttle (rpm)
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
