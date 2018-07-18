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

void PlanePlugin::actuatorCallback(const gazebo_example::actuator::ConstPtr& actuator_msg)
{
  boost::mutex::scoped_lock scoped_lock(lock_);
  actuator_[0] = actuator_msg->u1;
  actuator_[1] = actuator_msg->u2;
  actuator_[2] = actuator_msg->u3;
  actuator_[3] = actuator_msg->u4;
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
    ros::SubscribeOptions::create<gazebo_example::actuator>("actuator", MAX_SUB_QUEUE_SIZE,
        boost::bind(&PlanePlugin::actuatorCallback, this, _1),
        ros::VoidPtr(), &queue_);
    
  actuator_sub_ = rosnode_->subscribe(so);
  
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
    aerodynamics();
    link_->AddRelativeForce(force_);
    link_->AddRelativeTorque(torque_);
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
  math::Vector3 angular_velocity = link_->GetWorldAngularVel();
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = velocity.x;
  twist_msg.linear.y = velocity.y;
  twist_msg.linear.z = velocity.z;
  
  twist_msg.angular.x = angular_velocity.x;
  twist_msg.angular.y = angular_velocity.y;
  twist_msg.angular.z = angular_velocity.z;
  
  twist_pub_.publish(twist_msg);
}

double PlanePlugin::saturate(double value, double min, double max)
{
  double output = value;
  if (value < min){output = min;}
  if (value > max){output = max;}
  return output;
}



void PlanePlugin::aerodynamics()
{ 
  math::Vector3 velocity_bg = link_->GetRelativeLinearVel();
  math::Vector3 angular_velocity_bg = link_->GetRelativeAngularVel();
  math::Vector3 velocity_b(velocity_bg[0],-velocity_bg[1],-velocity_bg[2]);
  math::Vector3 angular_velocity_b(angular_velocity_bg[0],-angular_velocity_bg[1],-angular_velocity_bg[2]);
  //saturate actuators and add rate limiters!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  double delta_a_max = 52.0;
  double delta_e_max = 59.0;
  double delta_r_max = 49.0;
  double omega_t_min = 1716.0;
  double omega_t_max = 6710.0;



  McFoamy_FM_v2(saturate(actuator_[0],-delta_a_max,delta_a_max)*.017453, saturate(actuator_[1],-delta_e_max,delta_e_max)*.017453, saturate(actuator_[2],-delta_r_max,delta_r_max)*.017453, saturate(actuator_[3],omega_t_min,omega_t_max), velocity_b.x,velocity_b.y,velocity_b.z,angular_velocity_b.x,angular_velocity_b.y,angular_velocity_b.z,&Fx_b,&Fy_b,&Fz_b,&Mx_b,&My_b,&Mz_b);
  //McFoamy_FM_v2(actuator_[0]*.017453, actuator_[1]*.017453, actuator_[2]*.017453, actuator_[3], velocity_b.x,velocity_b.y,velocity_b.z,angular_velocity_b.x,angular_velocity_b.y,angular_velocity_b.z,&Fx_b,&Fy_b,&Fz_b,&Mx_b,&My_b,&Mz_b);

  /*double dFx =4.5;
  double dFy=4.4;
  double dFz=300.3;
  double dMx=0.0;
  double dMy=0.0;
  double dMz=0.0;
  force2_ = math::Vector3(dFx, dFy, dFz);
  torque2_ = math::Vector3(dMx, dMy, dMz);*/
  //force_ = math::Vector3(Fx, Fy, Fz);
  force_ = math::Vector3(Fx_b, -Fy_b, -Fz_b); //negative signs account for gazebo body frame different from mine
  torque_ = math::Vector3(Mx_b, -My_b, -Mz_b);

}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(PlanePlugin);

}
