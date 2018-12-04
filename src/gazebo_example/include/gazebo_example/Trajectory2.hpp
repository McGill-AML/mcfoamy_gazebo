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
  double DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double yaw_offset, gazebo::math::Vector3 position_offset_i);
  gazebo::math::Vector3 TransformPointToCameraFrame(gazebo::math::Quaternion q, double yaw_offset, gazebo::math::Vector3 position_offset_i, int index);
  double DistanceToIntermediateGoal(double yaw_offset, gazebo::math::Vector3 position_offset_i, gazebo::math::Vector3 intermediate_goal_i);
  void SetNumberOfLines(const std::string& filename);
  int GetNumberOfLines();
  bool NoCollision(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double yaw_offset, gazebo::math::Vector3 position_offset_i);
  gazebo::math::Vector3 End_Position(double yaw_offset, gazebo::math::Vector3 position_offset_i);
  double End_Yaw(double yaw_offset);
  bool InFieldOfView(gazebo::math::Quaternion q, double yaw_offset, gazebo::math::Vector3 position_offset_i);


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
  
private:
  int parent;
  int index;
  std::vector<size_t> sorted_maneuvers;
  gazebo::math::Vector3 position_i;
  double yaw;

};

class TrajectoryLibrary

{
public:
  TrajectoryLibrary();
  void LoadLibrary(std::vector<std::string> filenames);
  Trajectory GetTrajectoryAtIndex(int index);
  int GetNumberOfTrajectories();
  std::vector<node> SelectTrajectories(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_initial_i,gazebo::math::Vector3 p_goal_i);

private:
  double max_distance;
  int max_distance_index;
  std::vector<Trajectory> trajectory_library;
  int number_of_trajectories; 

};