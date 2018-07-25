#include "gazebo_example/motion_planner.h"

MotionPlanner::MotionPlanner(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  cloud = cloud;

}


void MotionPlanner::NearestNeighbor(double searchpoint[3], double *closest_point_index, double *distance)
{
  pcl::PointXYZ search_point;
  search_point.x = searchpoint[0];
  search_point.y = searchpoint[1];
  search_point.z = searchpoint[2];

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  *distance = sqrt(kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance));
  *closest_point_index = pointIdxNKNSearch(1);
}

double MotionPlanner::DistanceToTrajectory(double trajectory[][13], double d_min, double d_max, )
{
  
}
