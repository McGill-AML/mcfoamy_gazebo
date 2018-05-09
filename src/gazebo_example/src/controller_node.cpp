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
  wrench_pub_ = node_handle.advertise<geometry_msgs::Wrench>("external_wrench", MAX_PUB_QUEUE);
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
  
  const double frequency = 10.0; 
  ros::Rate loop_rate(frequency);
  
  while(ros::ok())
  {
    // Handle Callbacks
    ros::spinOnce();
    
    // Compute and publish controller output
    wrench_pub_.publish(compute_control_wrench(frequency));
    
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

geometry_msgs::Wrench ControllerNode::compute_control_wrench(const double frequency)
{
  const controllers::State desired_state = {5.0, 0};
  const controllers::State measured_state = {pose_.position.z, twist_.linear.z};
  const double force_z = pid_.output(desired_state, measured_state, 1.0 / frequency);
  
  geometry_msgs::Wrench command_wrench;
  command_wrench.force.z = force_z;
  
  return command_wrench;
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
