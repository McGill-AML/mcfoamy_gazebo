#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>       /* sqrt */
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


class Trajectory

{
public:
  Trajectory(std::string filename_csv);
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




class node

{
public:
  node();
  void SetParent(int x);
  int GetParent();
  void SetIndex(int x);
  int GetIndex();
  void SetSortedManeuvers(std::vector<size_t> x);
  std::vector<size_t> GetSortedManeuvers();
  void SetPosition(gazebo::math::Vector3 x);
  gazebo::math::Vector3 GetPosition();
  void SetYaw(double x);
  double GetYaw();
  void SetMinDistToObst(double x);
  double GetMinDistToObst();
  void SetInFOV(bool x);
  bool GetInFOV();
  void SetCollision(bool x);
  bool GetCollision();
  void SetTrajectoryToNode(int x);
  int GetTrajectoryToNode();
  
private:
  int parent;
  int index;
  std::vector<size_t> sorted_maneuvers;
  gazebo::math::Vector3 position_i;
  double yaw;
  double MinDistToObst;
  bool InFOV;
  bool Collision;
  int TrajectoryToNode;
};

class TrajectoryLibrary

{
public:
  TrajectoryLibrary();
  void LoadLibrary(std::vector<std::string> filenames);
  Trajectory GetTrajectoryAtIndex(int index);
  int GetNumberOfTrajectories();
  std::vector<int> Rollout(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_initial_i,gazebo::math::Vector3 p_goal_i, std::vector<node> *nodes_out, std::vector<node> *final_nodes_out);
  void SelectTrajectories3(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_initial_i,gazebo::math::Vector3 p_goal_i, std::vector<node> *nodes_out, std::vector<node> *final_nodes_out);

private:
  double max_distance;
  int max_distance_index;
  std::vector<Trajectory> trajectory_library;
  int number_of_trajectories;

  float lambda;
  gazebo::math::Matrix3 C_cb; 

};