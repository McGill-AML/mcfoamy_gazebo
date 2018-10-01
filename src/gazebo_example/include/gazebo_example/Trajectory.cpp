/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Eitan Bulka, <eitan.bulka@mail.mcgill.ca> 2018
 *
 */

#include "Trajectory.hpp"


Trajectory::Trajectory(std::string filename_csv) {
    trajectory_number = -1;
    number_of_lines = GetNumberOfLines(filename_csv);
    LoadTrajectory(filename_csv, number_of_lines, data);
    dt = data(1,0) - data(0,0); //assume constant delta t in trajectory
    aircraft_geometry.push_back(gazebo::math::Vector3(0.1,0.0,0.0));
    aircraft_geometry.push_back(gazebo::math::Vector3(0.0,0.5,0.0));
    aircraft_geometry.push_back(gazebo::math::Vector3(0.0,-0.5,0.0));
    aircraft_geometry.push_back(gazebo::math::Vector3(-0.7,0.0,0.0));

  	d_min = 0.2;
    d_max = 1.0;
    max_speed = 10.0;

    lambda = 0.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));

}


void Trajectory::LoadTrajectory(const std::string& filename, int number_of_lines, Eigen::MatrixXd &matrix) {

    matrix.resize(number_of_lines,14); // minus 1 for header, i = number of columns

    int i =  0;
    int row_num = 0;

    CsvParser *csvparser = CsvParser_new(filename.c_str(), ",", 0);
    CsvRow *row;

    while ((row = CsvParser_getRow(csvparser)) ) {
      //printf("==NEW LINE==\n");
        const char **rowFields = CsvParser_getFields(row);
        for (i = 0 ; i < CsvParser_getNumFields(row) ; i++) {
            //printf("FIELD: %s\n", rowFields[i]);
            matrix(row_num,i) = atof(rowFields[i]);

        }
    //printf("\n");
        CsvParser_destroy_row(row);
        row_num ++;
    }
    CsvParser_destroy(csvparser);

}


int Trajectory::GetNumberOfLines(const std::string& filename) {
    int number_of_lines = 0;
    std::string line;
    std::ifstream myfile(filename.c_str());

    while (getline(myfile, line)) {
        ++number_of_lines;
    }

    return number_of_lines;
}

Eigen::VectorXd Trajectory::GetStateAtIndex(int index){
 	return data.row(index);
}

int Trajectory::GetIndexAtTime(double time){
  int index = int(time/dt);
  if (index > number_of_lines-1){
    index = number_of_lines-1;
  }
  return index;
}

Eigen::VectorXd Trajectory::GetStateAtTime(double time){
  return GetStateAtIndex(GetIndexAtTime(time));
}



gazebo::math::Vector3 Trajectory::TransformPointToCameraFrame(gazebo::math::Quaternion q, int index){
  gazebo::math::Matrix3 C_psi(cos(q.GetYaw()),-sin(q.GetYaw()),0.0,sin(q.GetYaw()),cos(q.GetYaw()),0.0,0.0,0.0,1.0);//rotate by yaw, but because gazebo uses forward-left-up
  Eigen::VectorXd state = GetStateAtIndex(index);
  gazebo::math::Vector3 p_psi(state(1),state(2),state(3));
  gazebo::math::Vector3 p_c = C_cb * q.GetAsMatrix3().Inverse() * C_psi * p_psi;
  //gazebo::math::Vector3 p_c = C_cb * q.GetAsMatrix3() * p_psi;
  return p_c;
}

double Trajectory::DistanceToTrajectory(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q)
{
  pcl::PointXYZ searchPoint;
  std::vector<int> closest_point_index(1);
  std::vector<float> closest_distance_squared(1);
  double distance = 100.0;

  int i = 0;
  searchPoint.x = 0.0;
  searchPoint.y = 0.0;
  searchPoint.z = 0.0;
  if (octree.getLeafCount() > 0){
    if (octree.nearestKSearch (searchPoint, 1, closest_point_index, closest_distance_squared) > 0){
      while(i < number_of_lines){

        p_c = TransformPointToCameraFrame(q,i); 
        searchPoint.x = p_c[0];
        searchPoint.y = p_c[1];
        searchPoint.z = p_c[2];

        octree.nearestKSearch (searchPoint, 1, closest_point_index, closest_distance_squared);
        if (sqrt(closest_distance_squared[0]) < distance){
          distance = sqrt(closest_distance_squared[0]);
        }
        // Don't keep searching for min distance because this trajectory is crashing anyways
        if (sqrt(closest_distance_squared[0]) < d_min/2.0){
          break;
        }/*else if (sqrt(closest_distance_squared[0]) < d_max/2.0){
          Eigen::VectorXd state = GetStateAtIndex(i);
          gazebo::math::Vector3 p_psi(state(1),state(2),state(3));
          gazebo::math::Matrix3 C_psi(cos(q.GetYaw()),-sin(q.GetYaw()),0.0,sin(q.GetYaw()),cos(q.GetYaw()),0.0,0.0,0.0,1.0);//rotate by negative yaw, but because gazebo uses forward-left-up, GetYaw function returns negative yaw

          gazebo::math::Quaternion q_r(state(4),state(5),state(6),state(7));
          for (int ii = 0; ii < aircraft_geometry.size(); ++i){
            gazebo::math::Vector3 new_search_point = C_cb * q.GetAsMatrix3() * C_psi * (q_r.GetAsMatrix3().Inverse() * aircraft_geometry[i] + p_psi);
            searchPoint.x = new_search_point[0];
            searchPoint.y = new_search_point[1];
            searchPoint.z = new_search_point[2];
            octree.nearestKSearch (searchPoint, 1, closest_point_index, closest_distance_squared);
            if (sqrt(closest_distance_squared[0]) < distance){
              distance = sqrt(closest_distance_squared[0]);
            }
          }
        }*/else
        {
          i = i + 1;
          //double t_free = sqrt(closest_distance_squared[0])/max_speed;
          //i = i + int(t_free / dt); // assuming fixed time step in trajectory
        }
        // Checking distance using orientation of aircraft because distance is less than wingspan/2
        /*if (sqrt(closest_distance_squared[0]) > d_min/2.0 && sqrt(closest_distance_squared[0]) < d_max/2.0){
          //use Vector3/Matrix3 to C_ib * aircraft geometry, find min distance

        }

        if (sqrt(closest_distance_squared[0]) > d_max/2.0){
          double t_free = sqrt(closest_distance_squared[0])/max_speed;
          i = i + int(t_free / dt); // assuming fixed time step in trajectory
        }*/

      }
    }
  }

  return distance;
  
}

TrajectoryLibrary::TrajectoryLibrary(){

}

void TrajectoryLibrary::LoadLibrary(std::vector<std::string> filenames){
	number_of_trajectories = filenames.size();
	for (int i = 0; i < number_of_trajectories; ++i){
		trajectory_library.push_back(Trajectory(filenames[i]));
	}
}

Trajectory TrajectoryLibrary::GetTrajectoryAtIndex(int index){
 	return trajectory_library[index];
}

int TrajectoryLibrary::SelectTrajectory(double safety_distance, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q){
	int i = 0;

	while (i < number_of_trajectories){
		if (trajectory_library[i].DistanceToTrajectory(octree,q) > safety_distance){
			return i;
			break;
		}
		++i;
	}
	return -1;
}



int TrajectoryLibrary::GetNumberOfTrajectories(){
  return number_of_trajectories;
}