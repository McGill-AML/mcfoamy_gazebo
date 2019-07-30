#include "gazebo_example/controller_node.h"


namespace gazebo_example
{
  
const unsigned int ControllerNode::MAX_PUB_QUEUE = 1;
const unsigned int ControllerNode::MAX_SUB_QUEUE = 1;
  
  
ControllerNode::ControllerNode():
  start_(false)
  , node_handle("")
  , pid_(0.0, 0.0, 0.0)
  , lp_Vs(200.0, 2.0)
{}

bool ControllerNode::init()
{
  //actuator_pub_ = node_handle.advertise<gazebo_example::actuator>("actuator", MAX_PUB_QUEUE);
  actuator_pub_ = node_handle.advertise<std_msgs::Float64MultiArray>("actuator", MAX_PUB_QUEUE);
  ref_pose_pub_ = node_handle.advertise<geometry_msgs::Pose>("ref_pose", MAX_PUB_QUEUE);
  ref_twist_pub_ = node_handle.advertise<geometry_msgs::Twist>("ref_twist", MAX_PUB_QUEUE);
  ref_plane_vis_pub_ = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", MAX_PUB_QUEUE );

  pose_sub_ = node_handle.subscribe("pose", MAX_SUB_QUEUE, 
                                    &ControllerNode::poseCallback, this);
  twist_sub_ = node_handle.subscribe("twist", MAX_SUB_QUEUE, 
                                     &ControllerNode::twistCallback, this);
  init_pose_sub_ = node_handle.subscribe("init_pose", MAX_SUB_QUEUE, 
                                    &ControllerNode::init_poseCallback, this);
  trajectory_sub_ = node_handle.subscribe("trajectory", MAX_SUB_QUEUE, 
                                     &ControllerNode::trajectoryCallback, this); 
  trajectories_sub_ = node_handle.subscribe("trajectories", MAX_SUB_QUEUE, 
                                     &ControllerNode::trajectoriesCallback, this); 
  start_service_ = node_handle.advertiseService("start_controller",
                                                &ControllerNode::start_controller,
                                                this);
  pid_ = controllers::PID(5.0, 5.0, 0.0);
  previous_trajectory = -2;
  trajectory = 0;
  trajectory_type = 0;
  current_trajectory = 0;

  trajectory_old = 0;
  trajectory_starttime = 0.0;
  omega_t_old = 0.0;
  maneuver_switch = false;
  delta_hi_i = 0.0;
  delta_pi_i.x = 0.0;
  delta_pi_i.y = 0.0;
  delta_pi_i.z = 0.0;
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

double ControllerNode::saturate(double value, double min, double max)
{
  double output = value;
  if (value < min){output = min;}
  if (value > max){output = max;}
  return output;
}

gazebo::math::Vector3 ControllerNode::GetReferencePosition(Eigen::VectorXd state, gazebo::math::Vector3 initial_position, double initial_yaw){
  gazebo::math::Matrix3 C_psi(cos(initial_yaw),-sin(initial_yaw),0.0,sin(initial_yaw),cos(initial_yaw),0.0,0.0,0.0,1.0);
  gazebo::math::Vector3 Position_unrotated(state(1),state(2),state(3));
  return C_psi*Position_unrotated + initial_position;
}

gazebo::math::Quaternion ControllerNode::GetReferenceQuaternion(Eigen::VectorXd state, double initial_yaw){
  gazebo::math::Quaternion Quaternion_unrotated(state(4),state(5),state(6),state(7));
  gazebo::math::Vector3 euler_unrotated = Quaternion_unrotated.GetAsEuler();
  return gazebo::math::Quaternion(euler_unrotated[0],euler_unrotated[1],euler_unrotated[2] + initial_yaw); //would add initial yaw but subtract because of gazebo coordinate system
}




std_msgs::Float64MultiArray ControllerNode::compute_control_actuation(const double frequency)
{

  gazebo::math::Vector3 p_i(pose_.position.x, pose_.position.y, pose_.position.z);
  gazebo::math::Quaternion q(pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  gazebo::math::Vector3 v_b(twist_.linear.x, twist_.linear.y, twist_.linear.z);
  gazebo::math::Vector3 omega_b(twist_.angular.x, twist_.angular.y, twist_.angular.z);
  gazebo::math::Matrix3 C_bi = q.GetAsMatrix3().Inverse();

  if (new_trajectories_recieved){
    new_trajectories_recieved = false;
    if (trajectory_type == 1){
      //previous trajectory is agile
      if (ros::Time::now().toSec() - trajectory_starttime > CA.GetAgileTrajectoryAtIndex(trajectory).GetStateAtIndex(CA.GetAgileTrajectoryAtIndex(trajectory).GetNumberOfLines() - 1)(0) || trajectories_.data[2]){
        //previous agile trajectory is finished
        if (trajectories_.data[0] != 2){
          trajectory_starttime = ros::Time::now().toSec() - 0.1;
          trajectory_type = trajectories_.data[0];
          trajectory = trajectories_.data[1];  
          initial_position.x = init_pose_.position.x;
          initial_position.y = init_pose_.position.y;
          initial_position.z = init_pose_.position.z;
          initial_quaternion.w = init_pose_.orientation.w;
          initial_quaternion.x = init_pose_.orientation.x;
          initial_quaternion.y = init_pose_.orientation.y;
          initial_quaternion.z = init_pose_.orientation.z;
          if (trajectory_type == 1 && trajectory == 1){
            //new trajectory is hover to cruise, need to fix yaw singularity by pitching down by 
            gazebo::math::Quaternion body_y_rotation(.7071,0.0,-.7071,0.0);
            body_y_rotation.Normalize();
            initial_quaternion = initial_quaternion * body_y_rotation;
          }
          maneuver_switch = true;
        }
      }
    }
    else if(trajectory_type == 0){
      if (trajectories_.data[0] != 2){
        trajectory_starttime = ros::Time::now().toSec() - 0.1;
        trajectory_type = trajectories_.data[0];
        trajectory = trajectories_.data[1];  
        initial_position.x = init_pose_.position.x;
        initial_position.y = init_pose_.position.y;
        initial_position.z = init_pose_.position.z;
        initial_quaternion.w = init_pose_.orientation.w;
        initial_quaternion.x = init_pose_.orientation.x;
        initial_quaternion.y = init_pose_.orientation.y;
        initial_quaternion.z = init_pose_.orientation.z;
        if (trajectory_type == 1 && trajectory == 1){
          //new trajectory is hover to cruise, need to fix yaw singularity by pitching down by 
          gazebo::math::Quaternion body_y_rotation(.7071,0.0,-.7071,0.0);
          body_y_rotation.Normalize();
          initial_quaternion = initial_quaternion * body_y_rotation;
        }
        maneuver_switch = true;
      }
    }

 
  }





  trajectory_time = ros::Time::now().toSec() - trajectory_starttime;
  gazebo::math::Vector3 p_ref_i;
  gazebo::math::Quaternion q_ref;
  gazebo::math::Vector3 v_ref_r;
  gazebo::math::Vector3 omega_ref_r;
  if (trajectory_type == 0){
    //trim maneuver
    TrimTrajectory curr_traj(CA.trim_trajectories.row(trajectory),10.0,trajectory);
    p_ref_i = curr_traj.GetPositionAtTime(trajectory_time, initial_position, initial_quaternion.GetYaw());// p_ref_i.z = -10.0;
    q_ref = curr_traj.GetQuaternionAtTime(trajectory_time, initial_quaternion.GetYaw());
    v_ref_r = curr_traj.GetVelocity();
    omega_ref_r = curr_traj.GetAngularVelocity();
  }

  if (trajectory_type == 1){
    //agile maneuver
    AgileTrajectory curr_traj = CA.GetAgileTrajectoryAtIndex(trajectory);
    p_ref_i = curr_traj.GetPosition(initial_quaternion.GetYaw(), initial_position, trajectory_time);
    q_ref = curr_traj.GetQuaternion(initial_quaternion.GetYaw(), trajectory_time);
    v_ref_r = curr_traj.GetVelocity(trajectory_time);
    omega_ref_r = curr_traj.GetAngularVelocity(trajectory_time);
   }




  gazebo::math::Matrix3 C_ri = q_ref.GetAsMatrix3().Inverse();


  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  //marker.type = visualization_msgs::Marker::ARROW;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource ="package://gazebo_example/meshes/plane.dae";
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position.x = p_ref_i.y;
  marker.pose.position.y = p_ref_i.x;
  marker.pose.position.z = -p_ref_i.z;
  // frames
  // g - gazebo (ENU), east, north, up
  // r - rotors imu frame (FLU), forward, left, up
  // b - px4 (FRD) forward, right down
  // n - px4 (NED) north, east, down
  gazebo::math::Quaternion q_br(0, 1, 0, 0);
  gazebo::math::Quaternion q_ng(0, 0.70711, 0.70711, 0);
  gazebo::math::Quaternion q_nb = q_ref;
  gazebo::math::Quaternion q_gb = q_ng.GetInverse()*q_nb;
  gazebo::math::Quaternion q_gr = q_gb*q_br;
  marker.pose.orientation.x = q_gr.x;
  marker.pose.orientation.y = q_gr.y;
  marker.pose.orientation.z = q_gr.z;
  marker.pose.orientation.w = q_gr.w; 
  ref_plane_vis_pub_.publish( marker );




  //Constants and Aircraft Properties
  double ro = 1.225f; //Air Density (kg/m^3)
  double A_prop = 0.0507f; //Propeller Area (m^2)
  double m = .45f; //Mass (kg)
  double S = 0.14274f; //Wing Area (m^2), yak54 = 0.14865
  double b = 0.864f; //Wing Span (m), yak54 = .82
  double cbar = 0.21f; //Mean Aerodynamic Chord (m), yak54 =.2107
  double Cl_delta_a = -0.0006777f; //Aileron Control Derivative Coefficient (/deg)
  double Cm_delta_e = -0.0117747f; //Elevator Control Derivative Coefficient (/deg)
  double Cn_delta_r = -0.0035663f; //Rudder Control Derivative Coefficient (/deg)
  double delta_a_max = 52.0f; //52.0fMaximum Aileron Deflection (deg)
  double delta_e_max = 59.0f; //35.0fMaximum Elevator Deflection (deg)
  double delta_r_max = 49.0f; //56.0fMaximum Rudder Deflection (deg)
  double omega_t_min = 1716.0f; //Minimum Thrust (RPM)
  double omega_t_max = 6710.0f; //Maximimum Thrust (RPM)
  double F_aero2 = -0.0157;
  double F_aero1 = 0.0524;
  double F_aero0 = -0.5583;
  gazebo::math::Matrix3 I_b(0.003922, 0.0, 0.000441, 0.0, 0.015940, 0.0, 0.000441, 0.0, .01934);


  double Kap = 160.0;
  double Kad = 8.0;
  double Kpp = 0.08;
  double Kpd = 0.2;//was .1, .15 has very small oscillation when hovering
  double Kpi = 0.0;
  double Khp = 5.0;//was 5.0 in exp
  double Khi = 0.5;
  double Kv = 3.0;
  double Kaero = 2.0;

  if (trajectory_type == 0){
    //Kpp = 0.0;
    //Kpi = 0.0;
    Khp = 0.0;
    Khi = 0.0;
    Kv = 6.0;
  }

  double F_min = 0.0;
  double F_max = 10.0;
  gazebo::math::Vector3 F_hat_r(1.0,0.0,0.0); // direction of body-fixed thrust
  gazebo::math::Quaternion q_des;

  //double dt = 1.0 / frequency;
  double dt = ros::Time::now().toSec() - last_time;
  last_time = ros::Time::now().toSec();


  //Thrust Controller----------------------------------------------------------------------------------------------------------------------------------
  if (maneuver_switch == true){delta_hi_i = 0.0;}
  double delta_hp_i = -p_ref_i[2]-(-p_i[2]); //Height error (m)
  delta_hi_i += delta_hp_i * dt; //Height error integral (ms)
  gazebo::math::Vector3 vdelta_hp_i(0.0 , 0.0 , - delta_hp_i);
  gazebo::math::Vector3 vdelta_hi_i(0.0 , 0.0 , - delta_hi_i);
  gazebo::math::Vector3 a_des_b = (C_bi * C_ri.Inverse() * v_ref_r - v_b) * Kv  + C_bi * (vdelta_hp_i * Khp + vdelta_hi_i * Khi);
  //gazebo::math::Vector3 a_des_b = (C_bi * C_ri.Inverse() * v_ref_r - v_b) * Kv  + C_bi * ((p_ref_i - p_i) * Khp + vdelta_hi_i * Khi);
  double F_aero = Kaero*(F_aero2*powf(v_b.GetLength(),2.0) + F_aero1*v_b.GetLength() + F_aero0);
  gazebo::math::Vector3 vF_aero(-F_aero, 0.0 , 0.0);
  gazebo::math::Vector3 g_i(0.0, 0.0, 9.81); //Acceleration due to Gravity (m/s^2)
  double F_b = (a_des_b * m -C_bi * g_i * m - vF_aero).Dot(F_hat_r);

  gazebo::math::Vector3 Theta(0.0,0.0,0.0);

  //Position Controller-----------------------------------------------------------------------------------------------------------------------------------
  delta_pi_i += (p_ref_i - p_i) * dt;
  Theta = F_hat_r.Cross(C_ri * (Kpp * (p_ref_i - p_i) + Kpi * delta_pi_i) + (v_ref_r - C_ri * C_bi.Inverse()*v_b) * Kpd); //determine rotation of q_ref
  /*Bound reference quaternion rotations to 45 degrees*/
  double Theta0 = saturate(Theta[0], -3.1415/4.0, 3.1415/4.0);
  double Theta1 = saturate(Theta[1], -3.1415/4.0, 3.1415/4.0);
  double Theta2 = saturate(Theta[2], -3.1415/4.0, 3.1415/4.0);

  gazebo::math::Quaternion q_x(cos(Theta0/2.0),sin(Theta0/2.0),0.0,0.0); //x-axis rotation
  gazebo::math::Quaternion q_y(cos(Theta1/2.0),0.0,sin(Theta1/2.0),0.0); //y-axis rotation
  gazebo::math::Quaternion q_z(cos(Theta2/2.0),0.0,0.0,sin(Theta2/2.0)); //z-axis rotation
  q_des = q_ref * q_z * q_y * q_x; //Desired Quaternion
  

  //Attitude Controller------------------------------------------------------------------------------------------------------------------------------------
  gazebo::math::Quaternion delta_q = q.GetInverse() * q_des; //Error Quaternion
  //gazebo::math::Quaternion delta_q = Mult(q.GetInverse(), q_des); //Error Quaternion
  delta_q.w = saturate(delta_q.w, -1.0, 1.0);
  if (delta_q.w < 0.0){delta_q = -delta_q;}
  gazebo::math::Vector3 E_b(0.0, 0.0, 0.0);
  double imaglength = pow(pow(delta_q.x,2.0)+pow(delta_q.y,2.0)+pow(delta_q.z,2.0),0.5);
  if (imaglength > 0.000001){
    E_b.x = 2.0*acos(delta_q.w)*delta_q.x/imaglength;
    E_b.y = 2.0*acos(delta_q.w)*delta_q.y/imaglength;
    E_b.z = 2.0*acos(delta_q.w)*delta_q.z/imaglength;
  }

  gazebo::math::Vector3 M_b = I_b * (Kap * E_b + Kad * (C_bi * C_ri.Inverse() * omega_ref_r - omega_b));

  //Mixer-----------------------------------------------------------------------------------------------------------------------------------------------------
  
  F_b = saturate(F_b, F_min, F_max);
  //if (F_b < 0.0){F_b = 0.0;}
  double Vs = sqrt(v_b[0] * v_b[0] + 2.0 * F_b /(ro * A_prop)); //Approximated Slipstream Velocity (m/s)
  double Vs_min = sqrt(2*m*9.81/(ro*A_prop)); //Approximated minimum slipstream velocity-- Calculated from hover
  //Ensure a minimum slipstream velocity
  double Vs_filt;
  if (Vs < Vs_min){Vs = Vs_min;}
  Vs_filt = lp_Vs.apply(Vs);
  if (Vs_filt < Vs_min){Vs_filt = Vs_min;}
         
  double delta_a = M_b[0]/(.5*ro*Vs_filt*Vs_filt*S*b*Cl_delta_a); //Aileron Deflection (deg)
  double delta_e = M_b[1]/(.5*ro*Vs_filt*Vs_filt*S*cbar*Cm_delta_e); //Elevator Deflection (deg)
  double delta_r = M_b[2]/(.5*ro*Vs_filt*Vs_filt*S*b*Cn_delta_r); //Rudder Deflection (deg)
  double T;
     
  double Vs_des_A = 0.0;
  double Vs_des_E = 0.0;
  double Vs_des_R = 0.0;
  if (abs(delta_a) > delta_a_max){Vs_des_A = sqrt(abs(M_b[0]/(.5*ro*S*b*Cl_delta_a*delta_a_max)));}
  if (abs(delta_e) > delta_e_max){Vs_des_E = sqrt(abs(M_b[1]/(.5*ro*S*cbar*Cm_delta_e*delta_e_max)));}
  if (abs(delta_r) > delta_r_max){Vs_des_R = sqrt(abs(M_b[2]/(.5*ro*S*b*Cn_delta_r*delta_r_max)));}
  double Vs_des = Vs_des_A;
  if (Vs_des < Vs_des_E){Vs_des = Vs_des_E;}
  if (Vs_des < Vs_des_R){Vs_des = Vs_des_R;}
  if (Vs_des > v_b[0] && Vs_des > 0.0){T = F_b + (ro*A_prop/2)*(Vs_des * Vs_des - v_b[0] * v_b[0]);}
  else {T = F_b;}        
    
  double J = v_b.GetLength()/(omega_t_old*.0042333); //Advance ratio = V/(omega/60 * D)
  J = saturate(J, 0.0, 0.5);
  double kt = (-1.43909969 * J*J - 2.21240323 * J + 2.24512051) * pow(10.0,-7.0);  
  double omega_t = sqrt(T/kt); //RPM command        
  omega_t = saturate(omega_t, omega_t_min, omega_t_max);
  omega_t_old = omega_t;

  std::vector<double> u;
  u.push_back(delta_a); //aileron (Rad)
  u.push_back(delta_e); //elevator (Rad)
  u.push_back(delta_r); //rudder (Rad)
  u.push_back(omega_t); //throttle (rpm)

  command_actuator.data = u;

  ref_pose_.position.x = p_ref_i[0];
  ref_pose_.position.y = p_ref_i[1];
  ref_pose_.position.z = p_ref_i[2];
  
  ref_pose_.orientation.w = q_ref.w;
  ref_pose_.orientation.x = q_ref.x;
  ref_pose_.orientation.y = q_ref.y;
  ref_pose_.orientation.z = q_ref.z;

  ref_twist_.linear.x = v_ref_r[0];
  ref_twist_.linear.y = v_ref_r[1];
  ref_twist_.linear.z = v_ref_r[2];
  
  ref_twist_.angular.x = omega_ref_r[0];
  ref_twist_.angular.y = omega_ref_r[1];
  ref_twist_.angular.z = omega_ref_r[2];

  ref_pose_pub_.publish(ref_pose_);
  ref_twist_pub_.publish(ref_twist_);

  previous_trajectory = trajectory_.data;
  maneuver_switch = false;
  
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

void ControllerNode::init_poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  init_pose_ = *msg;
}

/*void ControllerNode::init_twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  init_twist_ = *msg;
}*/

void ControllerNode::trajectoryCallback(const std_msgs::Int16::ConstPtr& msg)
{
  trajectory_ = *msg;
  //trajectory = trajectory_.data;
  new_trajectory_recieved = true;
}

void ControllerNode::trajectoriesCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  trajectories_ = *msg;
  //trajectory = trajectory_.data;
  
  new_trajectories_recieved = true;
}

bool gazebo_example::ControllerNode::start_controller(std_srvs::Trigger::Request& req,
                                                      std_srvs::Trigger::Response& res)
{
  if (start_ != true){
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_ATA.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_H2C.csv");
    filenames.push_back("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/7_C2H.csv");

    CA.LoadAgileLibrary(filenames);
    CA.LoadTrimTrajectories("/home/eitan/mcfoamy_gazebo/src/gazebo_example/include/gazebo_example/trajectory_csvs/trim_cond.csv");
    last_time = ros::Time::now().toSec();


  }
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
