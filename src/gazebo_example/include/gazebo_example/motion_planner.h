#pragma once

#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <iostream>
#include <vector>
#include <ctime>

class MotionPlanner
{
public:
	MotionPlanner(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud); 
	void NearestNeighbor(int TrajecoryID);
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  
};