#include "gazebo_example/motionplan_node.h"

namespace gazebo_example
{
  
const unsigned int MotionplanNode::MAX_PUB_QUEUE = 1;
const unsigned int MotionplanNode::MAX_SUB_QUEUE = 1;

  
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
  trajectories_pub_ = node_handle.advertise<std_msgs::Int16MultiArray>("trajectories", MAX_PUB_QUEUE);


  //traj_pcl_pub_ = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ>> ("traj_pcl", 1);

  pose_sub_ = node_handle.subscribe("pose", MAX_SUB_QUEUE, 
                                    &MotionplanNode::poseCallback, this);
  twist_sub_ = node_handle.subscribe("twist", MAX_SUB_QUEUE, 
                                     &MotionplanNode::twistCallback, this);
  points_sub_ = node_handle.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/camera/depth/points", 1, &MotionplanNode::callback, this);
  start_service_ = node_handle.advertiseService("start_motionplan",
                                                &MotionplanNode::start_motionplan,
                                                this);
  //vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  vis_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", MAX_PUB_QUEUE );

  return true;
}

void MotionplanNode::run()
{
  wait_for_trigger();
  
  const double frequency = 10.0; 
  ros::Rate loop_rate(frequency);
  
  while(ros::ok())
  {
    // Handle Callbacks
    ros::spinOnce();
    
    // Compute and publish controller output
    compute_refstate();
    trajectory_pub_.publish(trajectory_);
    trajectories_pub_.publish(trajectories_);
    trajectories_.data.clear();


    //pcl_conversions::toPCL(ros::Time::now(), traj_pcl_ptr_->header.stamp);

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
  gazebo::math::Vector3 p_i(pose_.position.x, pose_.position.y, pose_.position.z);
  gazebo::math::Quaternion q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> (.1);//resolution is 128,.1
  octree.setInputCloud (cloud);
  octree.addPointsFromInputCloud ();
  init_pose_ = pose_;
  init_pose_pub_.publish(init_pose_);


  //gazebo::math::Vector3 goal_position_i(15.0 * cos(ros::Time::now().toSec() / 5.0),15.0 * sin(ros::Time::now().toSec() / 5.0),-8.0);
  gazebo::math::Vector3 goal_position_i(30.0,0.0,-8.0);
  std::vector<gazebo::math::Vector3> positions_sampled;
  std::vector<TrimTrajectory> trajectories_sampled;

  //trajectory_.data = CA.SelectTrimTrajectory(p_i, q, octree, goal_position_i, &trajectories_sampled, &positions_sampled);
  std::vector<int> trajectory_packet = CA.SelectTrajectory(p_i, q, octree, goal_position_i, &trajectories_sampled, &positions_sampled, trajectory_packet_prev, restart_, ros::Time::now().toSec());
  trajectories_.data.push_back(trajectory_packet[0]);
  trajectories_.data.push_back(trajectory_packet[1]);
  trajectory_packet_prev = trajectory_packet;
  restart_ = false;
  


  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = -1;
  marker.type = visualization_msgs::Marker::SPHERE;

  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1; 

  visualization_msgs::MarkerArray planned_poses;
  if (1){


  for (int i = 0; i < positions_sampled.size(); ++i){
    marker.id += 1;    
    gazebo::math::Vector3 position_sample = positions_sampled[i]; //This is NED, but 'world' frame is ENU
    marker.pose.position.x = position_sample.y;
    marker.pose.position.y = position_sample.x;
    marker.pose.position.z = -position_sample.z;


    planned_poses.markers.push_back(marker);
  }
  
  marker.scale.x = 0.8;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.type = visualization_msgs::Marker::ARROW;

  for (int i = 0; i < trajectories_sampled.size(); ++i){  
    TrimTrajectory trajectory_sample = trajectories_sampled[i];
    if (trajectory_sample.index == trajectory_packet[1] && trajectory_packet[0] == 0){//selected trajectory
      marker.color.r = 0.0;
    } else{
      marker.color.r = 1.0;
    }
    for (float j = 0.0; j < trajectory_sample.delta_t; j = j + 0.5){
      gazebo::math::Vector3 position_sample = trajectory_sample.GetPositionAtTime(j, p_i, q.GetYaw());
      marker.id += 1;    
      marker.pose.position.x = position_sample.y;
      marker.pose.position.y = position_sample.x;
      marker.pose.position.z = -position_sample.z;

      gazebo::math::Quaternion q_ENU(0.0,0.0,3.1415/2.0 - trajectory_sample.GetQuaternionAtTime(j, q.GetYaw()).GetYaw());
      marker.pose.orientation.x = q_ENU.x;
      marker.pose.orientation.y = q_ENU.y;
      marker.pose.orientation.z = q_ENU.z;
      marker.pose.orientation.w = q_ENU.w;
      planned_poses.markers.push_back(marker);
    }
  }
  } 


  marker.id += 1;
  marker.pose.position.x = goal_position_i.y;
  marker.pose.position.y = goal_position_i.x;
  marker.pose.position.z = -goal_position_i.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  planned_poses.markers.push_back(marker);
  vis_pub_.publish(planned_poses);
  planned_poses.markers.clear(); 


  
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

  if (start_ != true){
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_ATA.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_H2C.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_C2H.csv");

    CA.LoadAgileLibrary(filenames);
    CA.LoadTrimTrajectories("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/trim_cond.csv");
    trajectory_packet_prev.push_back(1);
    trajectory_packet_prev.push_back(0);

  }
  start_ = true;
  res.success = true;
  restart_ = true;

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
