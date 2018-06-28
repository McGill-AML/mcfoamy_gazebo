#include "gazebo_example/motionplan_node.h"

namespace gazebo_example
{
  
const unsigned int MotionplanNode::MAX_PUB_QUEUE = 10;
const unsigned int MotionplanNode::MAX_SUB_QUEUE = 10;

  
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

MotionplanNode::MotionplanNode():
  start_(false)
  , node_handle("")
{}

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
  
  const double frequency = 100.0; 
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
  int i = 0;
  //double lambda = 0.0;
  //double Kg = 0.0;
  //double Ko = 500.0;
  //double Kr = 10.0;
  //gazebo::math::Vector3 G_a(1000.0 - pose_.position.x, 0.0 - -pose_.position.y, 10.0 - -pose_.position.z);

  //gazebo::math::Matrix3 _C_cb(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));

  gazebo::math::Vector3 _r_a(pose_.position.x, -pose_.position.y, -pose_.position.z);
  gazebo::math::Quaternion _q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  gazebo::math::Vector3 _v_a(twist_.linear.x, -twist_.linear.y, -twist_.linear.z);
  gazebo::math::Matrix3 _C_ba = _q.GetAsMatrix3();
  gazebo::math::Vector3 _v_b = _C_ba * _v_a;
  //gazebo::math::Vector3 _v_c = _C_cb * _C_ba * _v_a;
  double left_points = 0.0;
  double right_points = 0.0;
  double delta_psi;
  //gazebo::math::Vector3 Fo(0.0,0.0,0.0);
  //gazebo::math::Vector3 Fo1(0.0,0.0,0.0);
  //gazebo::math::Vector3 Fo2(0.0,0.0,0.0);
  //gazebo::math::Vector3 Fo3(0.0,0.0,0.0);
  //gazebo::math::Vector3 Fo4(0.0,0.0,0.0);
  //gazebo::math::Vector3 temp;
  //double max = 0.0;


  //double c1 = .00001;
  //double c2 = 4.0;
  //double c3 = 10.0;
  //double Fi = 0.0;
  BOOST_FOREACH (const pcl::PointXYZ& pt, points_)
  if (std::isnan(pt.x)){++i;}
  else{
    ++i;
    //gazebo::math::Vector3 ri_c(pt.x, pt.y, pt.z);
    if (pt.x<0.0){++left_points;}
    if (pt.x>0.0){++right_points;}

    //if (temp.x>0.0 && temp.y<0.0){Fo4 = Fo4 + temp;}

    //Fi =Fi -c1*_v_c.Normalize().Dot(ri_c.Normalize())*pow(1.0-ri_c.GetLength()/c3,2.0)/(1.0+pow(ri_c.GetLength()/c3,3.0));

    //Fo = Fo + (1.0-ri_c.GetLength()/Kr)/(1.0+ri_c.GetLength()/Kr) * (-ri_c.Normalize()+_v_c.Normalize()).Normalize();
    //Fo = Fo + _v_c.Normalize().Dot(ri_c.Normalize()) * (1.0-ri_c.GetLength()/Kr)/(1.0+ri_c.GetLength()/Kr) * (-ri_c.Normalize()-(_v_c.Normalize().Dot(ri_c.Normalize()))*_v_c.Normalize()).Normalize();
    //temp = _v_c.Normalize().Dot(ri_c.Normalize()) *  (-ri_c.Normalize()-(_v_c.Normalize().Dot(ri_c.Normalize()))*_v_c.Normalize()).Normalize();
    //temp = _v_c.Normalize().Dot(ri_c.Normalize()) *  (-ri_c.Normalize()-(_v_c.Normalize().Dot(ri_c.Normalize()))*_v_c.Normalize()).Normalize();

    //if (temp.x>0.0 && temp.y>0.0){Fo1 = Fo1 + temp;}
    //if (temp.x<0.0 && temp.y>0.0){Fo2 = Fo2 + temp;}
    //if (temp.x<0.0 && temp.y<0.0){Fo3 = Fo3 + temp;}
    //if (temp.x>0.0 && temp.y<0.0){Fo4 = Fo4 + temp;}

    //Fo = Fo + _v_c.Normalize().Dot(ri_c.Normalize()) *  (-ri_c.Normalize()-(_v_c.Normalize().Dot(ri_c.Normalize()))*_v_c.Normalize()).Normalize();

  }
    //double F = -c2 * _v_a.Normalize().Dot(G_a.Normalize()) + Fi;
    //Fo = Fo1;
    //if (Fo2.GetLength()> Fo.GetLength()){Fo = Fo2;}
    //if (Fo3.GetLength()> Fo.GetLength()){Fo = Fo3;}
    //if (Fo4.GetLength()> Fo.GetLength()){Fo = Fo4;}

  //gazebo::math::Vector3 F = Kg * _C_cb * _C_ba * G_a.Normalize() + Ko * 4.0*Fo/double(i);
  //gazebo::math::Vector3 Pw = _r_a*0.0 + _C_ba.Inverse() * _C_cb.Inverse() * F;

  //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  //printf ("\t(%f, %f, %f)\n", points_[10000].x, points_[10000].y, points_[10000].z);
  //printf ("%i \n", sum);
  double k = 20.0;
  if (left_points > right_points){delta_psi = k*left_points/i;}
  else{delta_psi = k*right_points/i;}
  p_global.x = cos(psi_global)*cos(psi_global) * (_r_a.x-p_0.x) + sin(psi_global)*cos(psi_global)*(_r_a.y-p_0.y) + p_0.x;
  p_global.y = sin(psi_global)*sin(psi_global) * (_r_a.y-p_0.y) + sin(psi_global)*cos(psi_global)*(_r_a.x-p_0.x) + p_0.y;

  refpose_.position.x = p_global.x;
  refpose_.position.y = p_global.y + u_global * (sin(psi_global + delta_psi) - sin(psi_global))/100.0;
  refpose_.position.z = p_final.z;
  
  gazebo::math::Quaternion q_ref(0.0,-theta_global,-(psi_global + delta_psi));

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

  /*refpose_.position.x = Pw.x;
  refpose_.position.y = Pw.y;
  refpose_.position.z = -10.0 + 0.0*Pw.z;
  
  refpose_.orientation.w = 1.0;
  refpose_.orientation.x = 0.0;
  refpose_.orientation.y = 0.0;
  refpose_.orientation.z = 0.0;
  
  
  reftwist_.linear.x = 5.0;
  reftwist_.linear.y = 0.0;
  reftwist_.linear.z = 0.0;
  
  reftwist_.angular.x = 0.0;
  reftwist_.angular.y = 0.0;
  reftwist_.angular.z = 0.0;*/
  
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
