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


class TrimTrajectory

{
public:
  TrimTrajectory(Eigen::VectorXd trim_states, float duration, int ind);
private:
  float psi_dot;
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
  float delta_t;
  int index;

  gazebo::math::Vector3 GetPositionAtTime(float delta_t, gazebo::math::Vector3 p, float psi);
  
};

class CollisionAvoidance

{
public:
  CollisionAvoidance();
  void LoadTrimTrajectories(const std::string& filename);
  int SelectTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial);
private:
  Eigen::MatrixXd trim_trajectories;
  TrimTrajectory get_trim_trajectory(gazebo::math::Vector3 p_initial, float psi_initial, gazebo::math::Vector3 p_final);
  float V;

  float lambda;
  gazebo::math::Matrix3 C_cb; 
  double HFOV;//for realsense d435
  double VFOV;//for realsense d435
  double range; //for realsense d435

  std::vector<gazebo::math::Vector3> get_final_positions_inertial(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial);


};
