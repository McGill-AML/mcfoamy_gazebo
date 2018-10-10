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
  init_pose_pub_ = node_handle.advertise<geometry_msgs::Pose>("init_pose", MAX_PUB_QUEUE);
  //reftwist_pub_ = node_handle.advertise<geometry_msgs::Twist>("reftwist", MAX_PUB_QUEUE);
  trajectory_pub_ = node_handle.advertise<std_msgs::Int16>("trajectory", MAX_PUB_QUEUE);

  //traj_pcl_pub_ = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>> ("traj_pcl", 1);

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
    init_pose_pub_.publish(init_pose_);
    trajectory_pub_.publish(trajectory_);

    pcl_conversions::toPCL(ros::Time::now(), traj_pcl_ptr_->header.stamp);

    //traj_pcl_pub_.publish(traj_pcl_ptr_);

    
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

  gazebo::math::Quaternion q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);

  trajectory_.data = Traj_Lib.SelectTrajectory(2.0,octree,q);
  

  /*traj_pcl_ptr_->header.frame_id = "plane_xacro_default/camera_link";
  //traj_pcl_ptr_->height = traj_pcl_ptr_->width = 1;
  traj_pcl_ptr_->points.clear();
  for (int j = 0; j<Traj_Lib.GetNumberOfTrajectories();){

  
  for (int i = 0; i < Traj_Lib.GetTrajectoryAtIndex(j).number_of_lines; i++){
    gazebo::math::Vector3 pos = Traj_Lib.GetTrajectoryAtIndex(j).TransformPointToCameraFrame(q, i);
   

  traj_pcl_ptr_->points.push_back (pcl::PointXYZ(pos[0], pos[1], pos[2]));
  }
  j= j + 1;
  }*/
  init_pose_ = pose_;
  
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
  /*p_0.x = pose_.position.x;
  p_0.y = -pose_.position.y;
  p_0.z = -pose_.position.z;
  p_final.x = 100.0;
  p_final.y = 0.0;
  p_final.z = -5.0;
  psi_global = atan((p_final.y-p_0.y)/(p_final.x-p_0.x));
  u_global = 5.0;
  theta_global = 0.0191*u_global*u_global - 0.3022*u_global + 1.3262;
  q_global.SetFromEuler({0.0,-theta_global,-psi_global});*/
  cloud->width = 1280;
  cloud->height = 720;
  //kdtree.setInputCloud (cloud);
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();

/*  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60.csv");*/
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_extended.csv");
  filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_extended.csv");
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
