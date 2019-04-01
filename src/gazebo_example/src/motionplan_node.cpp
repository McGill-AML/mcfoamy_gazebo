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
  vis_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 100 );

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

  //gazebo::math::Vector3 goal_position_i = p_i;
  //goal_position_i.y = goal_position_i.y + 8.0; 
  //trajectory_.data = Traj_Lib.SelectTrajectory2(octree,q,p_i);
  //trajectory_.data = 1;
  //std::vector<node> nodes = Traj_Lib.SelectTrajectories(octree,q,p_i,goal_position_i);
  //printf("%i\n", nodes.size());
  //gazebo::math::Vector3 goal_position_i(20.0,20.0,-15.0);
  //gazebo::math::Vector3 goal_position_i(15.0 * cos(ros::Time::now().toSec() / 5.0),15.0 * sin(ros::Time::now().toSec() / 5.0),-8.0);
  gazebo::math::Vector3 goal_position_i(60.0,0.0,-8.0);

  /*std::vector<node> nodes;
  std::vector<node> final_nodes;
  Traj_Lib.SelectTrajectories3(octree,q,p_i,goal_position_i, &nodes, &final_nodes);
  if (final_nodes.size() < 1){
    printf("%s\n", "no trajectory found without collision so assigning ATA");
    trajectory_.data = -1;
  }
  else{
    trajectory_.data = final_nodes[final_nodes.size() - 1].GetSortedManeuvers()[0];
  }
  for (int i = final_nodes.size() - 1; i > -1; --i){
    trajectories_.data.push_back(final_nodes[i].GetSortedManeuvers()[0]);
  }*/
  std::vector<node> nodes;
  std::vector<node> final_nodes;
  std::vector<int> trajectories = Traj_Lib.Rollout(octree,q,p_i,goal_position_i, &nodes, &final_nodes);
  for (int i = 0; i < trajectories.size(); i++){
    trajectories_.data.push_back(trajectories[i]);
    //printf("%i\n", trajectories[i]);
    //printf("%i\n", final_nodes[trajectories.size()-i].GetTrajectoryToNode());;
  }
  //printf("%i\n",trajectories.size() - final_nodes.size() );


  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = -1;
  marker.type = visualization_msgs::Marker::ARROW;

  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  visualization_msgs::MarkerArray planned_poses;
  for (int i = 0; i < nodes.size(); ++i){
    marker.id += 1;    
    gazebo::math::Vector3 node_pos_i = nodes[i].GetPosition(); //This is NED, but 'world' frame is ENU
    marker.pose.position.x = node_pos_i.y;
    marker.pose.position.y = node_pos_i.x;
    marker.pose.position.z = -node_pos_i.z;
    gazebo::math::Quaternion q_ENU(0.0,0.0,3.1415/2.0 - nodes[i].GetYaw());
    marker.pose.orientation.x = q_ENU.x;
    marker.pose.orientation.y = q_ENU.y;
    marker.pose.orientation.z = q_ENU.z;
    marker.pose.orientation.w = q_ENU.w; 

    planned_poses.markers.push_back(marker);
  }

  marker.scale.x = 0.8;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  for (int i = 0; i < final_nodes.size(); ++i){
    marker.id += 1;    
    gazebo::math::Vector3 final_node_pos_i = final_nodes[i].GetPosition(); //This is NED, but 'world' frame is ENU
    marker.pose.position.x = final_node_pos_i.y;
    marker.pose.position.y = final_node_pos_i.x;
    marker.pose.position.z = -final_node_pos_i.z;
    gazebo::math::Quaternion q_ENU(0.0,0.0,3.1415/2.0 - final_nodes[i].GetYaw());
    marker.pose.orientation.x = q_ENU.x;
    marker.pose.orientation.y = q_ENU.y;
    marker.pose.orientation.z = q_ENU.z;
    marker.pose.orientation.w = q_ENU.w; 

    planned_poses.markers.push_back(marker);
  }
  /*if (final_nodes.size() > 0){
    marker.id += 1;    
    gazebo::math::Vector3 final_node_pos_i = Traj_Lib.GetTrajectoryAtIndex(final_nodes[0].GetSortedManeuvers()[0]).End_Position(final_nodes[0].GetYaw(),final_nodes[0].GetPosition()); //This is NED, but 'world' frame is ENU
    marker.pose.position.x = final_node_pos_i.y;
    marker.pose.position.y = final_node_pos_i.x;
    marker.pose.position.z = -final_node_pos_i.z;
    gazebo::math::Quaternion q_ENU(0.0,0.0,3.1415/2.0 - Traj_Lib.GetTrajectoryAtIndex(final_nodes[0].GetSortedManeuvers()[0]).End_Yaw(final_nodes[0].GetYaw()));
    marker.pose.orientation.x = q_ENU.x;
    marker.pose.orientation.y = q_ENU.y;
    marker.pose.orientation.z = q_ENU.z;
    marker.pose.orientation.w = q_ENU.w; 
    marker.color.g = 1.0;

    planned_poses.markers.push_back(marker);
  }*/

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

  if (start_ != true){
    /*filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_extended4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_ATA.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/Climb2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/Climbn2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/Climb4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/Climbn4.csv");  
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/Climb6.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/Climbn6.csv");*/
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_ATA.csv");

    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_extended4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_0.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_-2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_-4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_6.csv");
    //ilenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_0_-6.csv");

    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_0.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_-2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_-4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_6.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_15_-6.csv");

    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_0.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_-2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_-4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_6.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-15_-6.csv");


    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_0.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_-2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_-4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_6.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_30_-6.csv");

    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_0.csv");
    /*filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_-2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_-4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_6.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-30_-6.csv");


    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_0.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_-2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_-4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_6.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_45_-6.csv");

    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_0.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_-2.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_-4.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_6.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-45_-6.csv");*/

    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_0.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_-2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_-4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_6.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_60_-6.csv");

    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_0.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_-2.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_-4.csv");
    //filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_-60_6.csv");





    Traj_Lib.LoadLibrary(filenames);

  }
  start_ = true;
  res.success = true;

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
