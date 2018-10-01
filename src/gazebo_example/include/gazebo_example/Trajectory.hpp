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
private:
  double max_distance;
  int max_distance_index;
  std::vector<Trajectory> trajectory_library;
  int number_of_trajectories; 

};