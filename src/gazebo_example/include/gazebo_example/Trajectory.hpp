#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>       /* sqrt */
#include "csvparser.c"

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
  void LoadTrajectory(const std::string& filename, int number_of_lines, Eigen::MatrixXd &matrix);
  Eigen::VectorXd GetStateAtIndex(int index);
  int GetIndexAtTime(double time);
  Eigen::VectorXd GetStateAtTime(double time);
  double DistanceToTrajectory(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q);
  gazebo::math::Vector3 TransformPointToCameraFrame(gazebo::math::Quaternion q, int index);
  int number_of_lines;

private:
  int trajectory_number;
  Eigen::MatrixXd data;
  int GetNumberOfLines(const std::string& filename);
  double dt;
  std::vector<gazebo::math::Vector3> aircraft_geometry;
  double d_min;
  double d_max;
  double max_speed;
  float lambda;
  gazebo::math::Matrix3 C_cb;
  gazebo::math::Vector3 p_c; //position in camera frame

};


class TrajectoryLibrary

{
public:
  TrajectoryLibrary();
  void LoadLibrary(std::vector<std::string> filenames);
  Trajectory GetTrajectoryAtIndex(int index);
  int SelectTrajectory(double safety_distance, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q);
  int GetNumberOfTrajectories();
  void SetGlobalLine(gazebo::math::Vector3 p_0, double psi_0);
  void DistanceToGlobalLine(gazebo::math::Vector3 p_i);
  void DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q);
  void HeadingToGlobalLine(gazebo::math::Quaternion q);
  void Cost(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_i);
  int SelectTrajectory2(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_i);
  void printC();

private:
  double max_distance;
  int max_distance_index;
  std::vector<Trajectory> trajectory_library;
  int number_of_trajectories; 
  gazebo::math::Vector3 p_0_global;
  double psi_0_global;
  std::vector<double> d_global;
  std::vector<double> d_obstacle;
  std::vector<double> d_psi;
  std::vector<double> C;

};

class node

{
public:
  void SetParent(int x);
  int GetParent();
  void SetIndex(int x);
  int GetIndex();
  void SetSortedManeuvers(std::vector<int> x);
  std::vector<int> GetSortedManeuvers();
  void SetPosition(gazebo::math::Vector3 x);
  gazebo::math::Vector3 GetPosition();
  
private:
  int parent;
  int index;
  std::vector<int> sorted_maneuvers;
  gazebo::math::Vector3 position;

};