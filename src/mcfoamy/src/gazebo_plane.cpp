#include <gazebo_example/gazebo_plane.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
PlanePlugin::PlanePlugin() :
  MAX_PUB_QUEUE_SIZE(20),
  MAX_SUB_QUEUE_SIZE(20)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PlanePlugin::~PlanePlugin()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void PlanePlugin::Load( physics::ModelPtr parent, sdf::ElementPtr sdf )
{
  ParseParameters(sdf);
  
  world_ = parent->GetWorld();
  link_ = parent->GetLink(this->robot_namespace_ +"_link");
  
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  
  InitROSNode();

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(
      boost::bind(&PlanePlugin::UpdateChild, this));
    
  ROS_INFO("Successful loading of the plane plugin with robot namespace: %s", robot_namespace_.c_str());
}

void PlanePlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void PlanePlugin::wrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock_);
  force_ = math::Vector3(wrench_msg->force.x, wrench_msg->force.y, wrench_msg->force.z);
  torque_ = math::Vector3(wrench_msg->torque.x, wrench_msg->torque.y, wrench_msg->torque.z);
}


void PlanePlugin::ParseParameters(sdf::ElementPtr sdf)
{
  robot_namespace_ = "plane_node";
  if (!sdf->HasElement("robotNamespace"))
  {
    ROS_WARN_NAMED("PlanePlugin", "PlanePlugin missing <robotNamespace>, "
        "defaults to \"%s\"", robot_namespace_.c_str());
  }
  else
  {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  }
}

void PlanePlugin::InitROSNode()
{
  rosnode_.reset(new ros::NodeHandle(robot_namespace_));
  
  pose_pub_ = rosnode_->advertise<geometry_msgs::Pose>("pose", MAX_PUB_QUEUE_SIZE );
  twist_pub_ = rosnode_->advertise<geometry_msgs::Twist>("twist", MAX_PUB_QUEUE_SIZE );
  
  // subscribe to the odometry topic
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<geometry_msgs::Wrench>("external_wrench", MAX_SUB_QUEUE_SIZE,
        boost::bind(&PlanePlugin::wrenchCallback, this, _1),
        ros::VoidPtr(), &queue_);
    
  wrench_sub_ = rosnode_->subscribe(so);
  
  // start custom queue for ROS message
  callback_queue_thread_ =
    boost::thread(boost::bind(&PlanePlugin::QueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void PlanePlugin::UpdateChild()
{
  {
    boost::mutex::scoped_lock scoped_lock(lock_);
    link_->AddForce(force_);
    link_->AddTorque(torque_);
  }
  
  publishLinkState();
}

void PlanePlugin::publishLinkState()
{
  math::Pose pose = link_->GetWorldPose();  
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose.pos.x;
  pose_msg.position.y = pose.pos.y;
  pose_msg.position.z = pose.pos.z;
  
  pose_msg.orientation.w = pose.rot.w;
  pose_msg.orientation.x = pose.rot.x;
  pose_msg.orientation.y = pose.rot.y;
  pose_msg.orientation.z = pose.rot.z;
  
  pose_pub_.publish(pose_msg);
  
  math::Vector3 velocity = link_->GetWorldLinearVel();
  math::Vector3 angular_velocity = link_->GetWorldAngularAccel();
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = velocity.x;
  twist_msg.linear.y = velocity.y;
  twist_msg.linear.z = velocity.z;
  
  twist_msg.angular.x = angular_velocity.x;
  twist_msg.angular.y = angular_velocity.y;
  twist_msg.angular.z = angular_velocity.z;
  
  twist_pub_.publish(twist_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(PlanePlugin);

}
