#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>       
#include "csvparser.c"
#include "sort.h"

#include <stdio.h>

#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <Eigen/Core>

#include <iostream>
#include <vector>
#include <ctime>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#define PI 3.1415f


class TrimTrajectory

{
public:
  TrimTrajectory(Eigen::VectorXd trim_states, float duration, int ind);
  float DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q_initial, gazebo::math::Vector3 p_initial, float delta_t);
  float delta_t;
  int index;
  float psi_dot_deg;
  float z_dot;
  float psi_dot;


  gazebo::math::Vector3 GetPositionAtTime(float t, gazebo::math::Vector3 p, float psi);
  gazebo::math::Quaternion GetQuaternionAtTime(float t, float psi);
  gazebo::math::Vector3 GetVelocity();
  gazebo::math::Vector3 GetAngularVelocity();
private:
  float phi;
  float theta;
  float u;
  float v;
  float w;
  float p;
  float q;
  float r;

  float V;
  float V_xy;
  float lambda;
  gazebo::math::Matrix3 C_cb; 
  double d_max;
};

class AgileTrajectory

{
public:
  AgileTrajectory(std::string filename_csv);
  void LoadTrajectory(const std::string& filename, Eigen::MatrixXd &matrix);
  Eigen::VectorXd GetStateAtIndex(int index);
  int GetIndexAtTime(double time);
  Eigen::VectorXd GetStateAtTime(double time);
  double GetFinalTime();
  double DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i);
  gazebo::math::Vector3 TransformPointToCameraFrame(gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, int index);
  double DistanceToIntermediateGoal(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 intermediate_goal_i);
  double YawDistanceToGoal(double psi_node, gazebo::math::Vector3 p_i, gazebo::math::Vector3 p_goal_i);
  double AngleDistanceToGoal(double psi_node, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i);
  double AngleDistanceToGoal2(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i);
  double DistanceToGoal(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i);
  void SetNumberOfLines(const std::string& filename);
  int GetNumberOfLines();
  bool NoCollision(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i);
  gazebo::math::Vector3 End_Position(double psi_node, gazebo::math::Vector3 p_node_i);
  double End_Yaw(double psi_node);
  gazebo::math::Quaternion End_Quaternion(double psi_node);
  bool InFieldOfView(gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i);
  gazebo::math::Vector3 GetPosition(double psi_node, gazebo::math::Vector3 p_node_i, double t);
  gazebo::math::Vector3 GetPositionFromIndex(double psi_node, gazebo::math::Vector3 p_node_i, int k);
  gazebo::math::Quaternion GetQuaternion(double psi_node, double t);
  gazebo::math::Vector3 GetVelocity(double t);
  gazebo::math::Vector3 GetAngularVelocity(double t);

private:
  int trajectory_number;
  Eigen::MatrixXd data;
  double dt;
  std::vector<gazebo::math::Vector3> aircraft_geometry;
  double d_min;
  double d_max;
  double max_speed;
  float lambda;
  gazebo::math::Matrix3 C_cb;
  gazebo::math::Vector3 p_c; //position in camera frame
  int number_of_lines;
};

class CollisionAvoidance

{
public:
  CollisionAvoidance();
  void LoadTrimTrajectories(const std::string& filename);
  int SelectTrimTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Vector3 p_goal, std::vector<TrimTrajectory> *trajectories_out, std::vector<gazebo::math::Vector3> *final_positions_inertial_out, std::vector<int> trajectory_packet_prev);
  Eigen::MatrixXd trim_trajectories;
  void LoadAgileLibrary(std::vector<std::string> filenames);
  AgileTrajectory GetAgileTrajectoryAtIndex(int index);
  int GetNumberOfAgileTrajectories();
  std::vector<int> SelectTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Vector3 p_goal, std::vector<TrimTrajectory> *trajectories_out, std::vector<gazebo::math::Vector3> *final_positions_inertial_out, std::vector<int> trajectory_packet_prev, bool restart, double time);

private:
  TrimTrajectory get_trim_trajectory(gazebo::math::Vector3 p_initial, float psi_initial, gazebo::math::Vector3 p_final);
  float V;
  //int max_turn_rate;

  float lambda;
  gazebo::math::Matrix3 C_cb; 
  float HFOV;//for realsense d435
  float VFOV;//for realsense d435
  float range; //for realsense d435
  double d_max;

  std::vector<gazebo::math::Vector3> get_final_positions_inertial(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial);

  double max_distance;
  int max_distance_index;
  std::vector<AgileTrajectory> agile_trajectory_library;
  int number_of_agile_trajectories;
  int mode; //for finite state machine. 0 = start_hover, 1 = H2C, 2 = avoidance, 4 = C2H, 5 = goal_hover
  //int maneuver_type; // 0 = trim, 1 = agile
  int ata_count;
  double mission_start_time;
};


