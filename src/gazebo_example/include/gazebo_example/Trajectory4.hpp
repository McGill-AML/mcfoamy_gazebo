/*#include <iostream>
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
#include <pcl/octree/octree_search.h>*/
#define PI 3.1415f


class TrimTrajectory

{
public:
  TrimTrajectory(Eigen::VectorXd trim_states, float duration, int ind);
  float DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q_initial, gazebo::math::Vector3 p_initial, float delta_t);
  float delta_t;
  int index;

  gazebo::math::Vector3 GetPositionAtTime(float t, gazebo::math::Vector3 p, float psi);
  gazebo::math::Quaternion GetQuaternionAtTime(float t, float psi);
  gazebo::math::Vector3 GetVelocity();
  gazebo::math::Vector3 GetAngularVelocity();
private:
  float psi_dot;
  float psi_dot_deg;
  float z_dot;
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

class CollisionAvoidance

{
public:
  CollisionAvoidance();
  void LoadTrimTrajectories(const std::string& filename);
  int SelectTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Vector3 p_goal, std::vector<TrimTrajectory> *trajectories_out, std::vector<gazebo::math::Vector3> *final_positions_inertial_out);
  Eigen::MatrixXd trim_trajectories;
private:
  TrimTrajectory get_trim_trajectory(gazebo::math::Vector3 p_initial, float psi_initial, gazebo::math::Vector3 p_final);
  float V;

  float lambda;
  gazebo::math::Matrix3 C_cb; 
  float HFOV;//for realsense d435
  float VFOV;//for realsense d435
  float range; //for realsense d435
  double d_max;

  std::vector<gazebo::math::Vector3> get_final_positions_inertial(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial);
  
};
