#include <pcl/point_cloud.h>
#include "pcl/octree/octree_search.h"

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


class Trajectory

{
public:
  Trajectory(std::string filename_csv);
  void LoadTrajectory(const std::string& filename, int number_of_lines, Eigen::MatrixXd &matrix);
  Eigen::VectorXd GetStateAtIndex(int index);
  double DistanceToTrajectory(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree);
private:
  int trajectory_number;
  Eigen::MatrixXd data;
  int GetNumberOfLines(const std::string& filename);
  int number_of_lines;
  double dt;
  double aircraft_geometry[4][3];
  double d_min;
  double d_max;
  double max_speed;

};


class TrajectoryLibrary

{
public:
  TrajectoryLibrary();
  void LoadLibrary(std::vector<std::string> filenames);
  Trajectory GetTrajectoryAtIndex(int index);
  int SelectTrajectory(double safety_distance, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree);
  int GetNumberOfTrajectories();
private:
  double max_distance;
  int max_distance_index;
  std::vector<Trajectory> trajectory_library;
  int number_of_trajectories; 

};