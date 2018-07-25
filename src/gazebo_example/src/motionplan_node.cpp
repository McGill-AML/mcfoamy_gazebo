#include "gazebo_example/motionplan_node.h"

namespace gazebo_example
{
  
const unsigned int MotionplanNode::MAX_PUB_QUEUE = 10;
const unsigned int MotionplanNode::MAX_SUB_QUEUE = 10;

  
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

MotionplanNode::MotionplanNode():
  start_(false)
  , node_handle("")
{

}

bool MotionplanNode::init()
{
  refpose_pub_ = node_handle.advertise<geometry_msgs::Pose>("refpose", MAX_PUB_QUEUE);
  reftwist_pub_ = node_handle.advertise<geometry_msgs::Twist>("reftwist", MAX_PUB_QUEUE);

  pose_sub_ = node_handle.subscribe("pose", MAX_SUB_QUEUE, 
                                    &MotionplanNode::poseCallback, this);
  twist_sub_ = node_handle.subscribe("twist", MAX_SUB_QUEUE, 
                                     &MotionplanNode::twistCallback, this);
  points_sub_ = node_handle.subscribe<pcl::PointCloud<pcl::PointXYZ>>("points", 1, &MotionplanNode::callback, this);
  start_service_ = node_handle.advertiseService("start_motionplan",
                                                &MotionplanNode::start_motionplan,
                                                this);

  return true;
}

void MotionplanNode::run()
{
  wait_for_trigger();
  
  const double frequency = 50.0; 
  ros::Rate loop_rate(frequency);
  
  while(ros::ok())
  {
    // Handle Callbacks
    ros::spinOnce();
    
    // Compute and publish controller output
    compute_refstate();
    refpose_pub_.publish(refpose_);
    reftwist_pub_.publish(reftwist_);

    
    // Wait to maintain constant frequency
    loop_rate.sleep();
  }
}

void MotionplanNode::wait_for_trigger()
{
  while(ros::ok())
  {
    ros::spinOnce();
    if(start_) {break;}
    ROS_INFO_THROTTLE(10, "Waiting for motionplan start trigger");
  }
}

void MotionplanNode::compute_refstate()
{

  gazebo::math::Vector3 _r_a(pose_.position.x, -pose_.position.y, -pose_.position.z);
  gazebo::math::Quaternion _q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  gazebo::math::Vector3 _v_a(twist_.linear.x, -twist_.linear.y, -twist_.linear.z);
  gazebo::math::Matrix3 _C_ba = _q.GetAsMatrix3();
  gazebo::math::Vector3 _v_b = _C_ba * _v_a;

  float lambda = 0.0;
  gazebo::math::Matrix3 _C_cb(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
  gazebo::math::Matrix3 _C_ca = _C_cb * _C_ba;


  gazebo::math::Matrix3 _C_al(cos(-_q.GetYaw()),-sin(-_q.GetYaw()),0.0,sin((-_q.GetYaw())),cos((-_q.GetYaw())),0.0,0.0,0.0,1.0);


  printf("%f\n", Traj_Lib.GetTrajectoryAtIndex(0).DistanceToTrajectory(octree));
  printf("%i\n", Traj_Lib.SelectTrajectory(2.0,octree));


  p_global.x = cos(psi_global)*cos(psi_global) * (_r_a.x-p_0.x) + sin(psi_global)*cos(psi_global)*(_r_a.y-p_0.y) + p_0.x;
  p_global.y = sin(psi_global)*sin(psi_global) * (_r_a.y-p_0.y) + sin(psi_global)*cos(psi_global)*(_r_a.x-p_0.x) + p_0.y;

  refpose_.position.x = p_global.x;
  refpose_.position.y = p_global.y;
  refpose_.position.z = p_final.z;
  
  gazebo::math::Quaternion q_ref(0.0,-theta_global,-(psi_global));

  refpose_.orientation.w = q_ref.w;
  refpose_.orientation.x = q_ref.x;
  refpose_.orientation.y = q_ref.y;
  refpose_.orientation.z = q_ref.z;

  reftwist_.linear.x = u_global;
  reftwist_.linear.y = 0.0;
  reftwist_.linear.z = 0.0;
  
  reftwist_.angular.x = 0.0;
  reftwist_.angular.y = 0.0;
  reftwist_.angular.z = 0.0;


  
}


void MotionplanNode::poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  pose_ = *msg;
}

void MotionplanNode::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  twist_ = *msg;
}

void MotionplanNode::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  points_ = *msg;


  //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

bool gazebo_example::MotionplanNode::start_motionplan(std_srvs::Trigger::Request& req,
                                                      std_srvs::Trigger::Response& res)
{
  start_ = true;
  res.success = true;
  p_0.x = pose_.position.x;
  p_0.y = -pose_.position.y;
  p_0.z = -pose_.position.z;
  p_final.x = 100.0;
  p_final.y = 0.0;
  p_final.z = -5.0;
  psi_global = atan((p_final.y-p_0.y)/(p_final.x-p_0.x));
  u_global = 5.0;
  theta_global = 0.0191*u_global*u_global - 0.3022*u_global + 1.3262;
  q_global.SetFromEuler({0.0,-theta_global,-psi_global});
  cloud->width = 1280;
  cloud->height = 720;
  //kdtree.setInputCloud (cloud);
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/straight.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/straight.csv");
  Traj_Lib.LoadLibrary(filenames);


}

} // gazebo_example namespace



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motionplan_node");
  
  gazebo_example::MotionplanNode node;
  if(!node.init())
  {
    return 0;
  }
  
  node.run();

  return 0;
}
