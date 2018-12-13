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
  link_ = parent->GetLink("airframe");
  
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


void PlanePlugin::actuatorCallback(const std_msgs::Float64MultiArray::ConstPtr& actuator_msg)
{

  int i = 0;
  // print all the remaining numbers
  for(std::vector<double>::const_iterator it = actuator_msg->data.begin(); it != actuator_msg->data.end(); ++it)
  {
    actuator_[i] = *it;
    i++;
  }

  return;
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
    ros::SubscribeOptions::create<std_msgs::Float64MultiArray>("actuator", MAX_SUB_QUEUE_SIZE,
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
    //link_->AddRelativeForce(force_);
    //link_->AddRelativeTorque(torque_);
    link_->AddForce(force_);
    link_->AddTorque(torque_);
  }
  
  publishLinkState();
}

void PlanePlugin::publishLinkState()
{
  // frames
  // g - gazebo (ENU), east, north, up
  // r - rotors imu frame (FLU), forward, left, up
  // b - px4 (FRD) forward, right down
  // n - px4 (NED) north, east, down
  math::Quaternion q_gr = link_->GetWorldPose().rot;
  math::Quaternion q_br(0, 1, 0, 0);
  math::Quaternion q_ng(0, 0.70711, 0.70711, 0);

  math::Quaternion q_gb = q_gr*q_br.GetInverse();
  q_nb = q_ng*q_gb;

  math::Vector3 pos_g = link_->GetWorldPose().pos;

  pos_n = q_ng.RotateVector(pos_g);

  vel_b = q_br.RotateVector(link_->GetRelativeLinearVel());
  omega_nb_b = q_br.RotateVector(link_->GetRelativeAngularVel());

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pos_n.x;
  pose_msg.position.y = pos_n.y;
  pose_msg.position.z = pos_n.z;
  pose_msg.orientation.w = q_nb.w;
  pose_msg.orientation.x = q_nb.x;
  pose_msg.orientation.y = q_nb.y;
  pose_msg.orientation.z = q_nb.z;  
  pose_pub_.publish(pose_msg);

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = vel_b.x;
  twist_msg.linear.y = vel_b.y;
  twist_msg.linear.z = vel_b.z;
  twist_msg.angular.x = omega_nb_b.x;
  twist_msg.angular.y = omega_nb_b.y;
  twist_msg.angular.z = omega_nb_b.z;
  twist_pub_.publish(twist_msg);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pos_g.x, pos_g.y, pos_g.z) );
  tf::Quaternion q(q_gr.x, q_gr.y, q_gr.z, q_gr.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "airframe"));

  tf::Transform transform2;
  transform2.setOrigin( tf::Vector3(0.0,0.0,0.0) );
  tf::Quaternion q2(0.5,-0.5,0.5,-0.5);
  transform2.setRotation(q2);
  br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "kinect", "kinect_camera_frame"));

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
  /*math::Vector3 velocity_bg = link_->GetRelativeLinearVel();
  math::Vector3 angular_velocity_bg = link_->GetRelativeAngularVel();
  math::Vector3 velocity_b(velocity_bg[0],-velocity_bg[1],-velocity_bg[2]);
  math::Vector3 angular_velocity_b(angular_velocity_bg[0],-angular_velocity_bg[1],-angular_velocity_bg[2]);*/
  //saturate actuators and add rate limiters!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  double delta_a_max = 52.0;
  double delta_e_max = 59.0;
  double delta_r_max = 49.0;
  double omega_t_min = 1716.0;
  double omega_t_max = 6710.0;

  McFoamy_FM_v2(saturate(actuator_[0],-delta_a_max,delta_a_max)*.017453, saturate(actuator_[1],-delta_e_max,delta_e_max)*.017453, saturate(actuator_[2],-delta_r_max,delta_r_max)*.017453, saturate(actuator_[3],omega_t_min,omega_t_max), vel_b.x,vel_b.y,vel_b.z,omega_nb_b.x,omega_nb_b.y,omega_nb_b.z,&Fx_b,&Fy_b,&Fz_b,&Mx_b,&My_b,&Mz_b);
  //math::Quaternion q_br(0, 1, 0, 0);
  //force_ = q_br.RotateVectorReverse(math::Vector3(Fx_b, Fy_b, Fz_b)); 
  //torque_ = q_br.RotateVectorReverse(math::Vector3(Mx_b, My_b, Mz_b));

  math::Quaternion q_gr = link_->GetWorldPose().rot;
  math::Quaternion q_br(0, 1, 0, 0);
  math::Quaternion q_ng(0, 0.70711, 0.70711, 0);

  math::Quaternion q_gb = q_gr*q_br.GetInverse();
  force_ = q_gb.RotateVector(math::Vector3(Fx_b, Fy_b, Fz_b)); 
  torque_ = q_gb.RotateVector(math::Vector3(Mx_b, My_b, Mz_b));

  /*
  McFoamy_FM_v2(saturate(actuator_[0],-delta_a_max,delta_a_max)*.017453, saturate(actuator_[1],-delta_e_max,delta_e_max)*.017453, saturate(actuator_[2],-delta_r_max,delta_r_max)*.017453, saturate(actuator_[3],omega_t_min,omega_t_max), velocity_b.x,velocity_b.y,velocity_b.z,angular_velocity_b.x,angular_velocity_b.y,angular_velocity_b.z,&Fx_b,&Fy_b,&Fz_b,&Mx_b,&My_b,&Mz_b);

  force_ = math::Vector3(Fx_b, -Fy_b, -Fz_b); //negative signs account for gazebo body frame different from mine
  torque_ = math::Vector3(Mx_b, -My_b, -Mz_b);*/
 

}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(PlanePlugin);

}