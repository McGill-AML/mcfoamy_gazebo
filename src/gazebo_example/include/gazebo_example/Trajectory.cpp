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
    d_global.push_back(0.0);
    d_obstacle.push_back(0.0);
    d_psi.push_back(0.0);
    C.push_back(0.0);
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

void TrajectoryLibrary::SetGlobalLine(gazebo::math::Vector3 p_0, double psi_0){
  p_0_global = p_0;
  psi_0_global = psi_0;
}

void TrajectoryLibrary::DistanceToGlobalLine(gazebo::math::Vector3 p_i){
  Eigen::VectorXd state;

  for (int i = 0; i < number_of_trajectories; ++i){
    state = trajectory_library[i].GetStateAtIndex(trajectory_library[i].number_of_lines-1);
    gazebo::math::Vector3 p_end(state(1),state(2),state(3));
    p_end = p_end + p_i;
    d_global[i] = sqrt(pow(-sin(psi_0_global)*(p_end-p_0_global).x + cos(psi_0_global)*(p_end-p_0_global).y,2.0) + pow(p_end.z - p_0_global.z,2.0));
  } 
}

void TrajectoryLibrary::DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q){
  
  for (int i = 0; i < number_of_trajectories; ++i){
    d_obstacle[i] = trajectory_library[i].DistanceToTrajectory(octree,q);
  }
}

void TrajectoryLibrary::HeadingToGlobalLine(gazebo::math::Quaternion q){
  Eigen::VectorXd state;
  double psi_end;
  double delta_psi;

  for (int i = 0; i < number_of_trajectories; ++i){
    state = trajectory_library[i].GetStateAtIndex(trajectory_library[i].number_of_lines-1);
    gazebo::math::Quaternion q_end(state(4),state(5),state(6),state(7));
    psi_end = q.GetYaw() + q_end.GetYaw();
    delta_psi = psi_end - psi_0_global;
    if (delta_psi > 3.14){delta_psi = delta_psi - 2.0*3.14;}
    if (delta_psi < -3.14){delta_psi = delta_psi + 2.0*3.14;}

    d_psi[i] = fabs(delta_psi);

  } 
}

void TrajectoryLibrary::Cost(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_i){
  DistanceToGlobalLine(p_i);
  DistanceToObstacle(octree,q);
  HeadingToGlobalLine(q);
  double C1 = 100000.0;
  double C2 = 100.0;
  double C3 = 0.0;
  double d_crash = 0.5;
  double d_far = 3.0;
  double cost;

  for (int i = 0; i < number_of_trajectories; ++i){
    if (d_obstacle[i] < d_crash){
      cost = C1 + d_global[i] + C3*d_psi[i];
    }
    else if (d_obstacle[i] < d_far){
      cost = C2 * (d_obstacle[i] - d_crash) + d_global[i] + C3*d_psi[i];
    }
    else{
      cost = d_global[i] + C3*d_psi[i];
    }

    C[i] = cost;
    //printf("%i\n", i);
    //printf("%f\n",d_obstacle[i] );
    //printf("%f\n",d_global[i] );
    //printf("%f\n",d_psi[i] );
  } 
}



int TrajectoryLibrary::SelectTrajectory2(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_i){
  double lowest_cost = C[0];
  int best_trajectory = 0;
  Cost(octree,q,p_i);

  for (int i = 0; i < number_of_trajectories; ++i){
    if (C[i] < lowest_cost){
      lowest_cost = C[i];
      best_trajectory = i;
    }
  }
  return best_trajectory; 
}

void TrajectoryLibrary::printC(){
  for (int i = 0; i < number_of_trajectories; i++){
printf("%f\n", float(C[i]));
printf("%f\n", float(d_obstacle[i]));

}
}

void node::SetParent(int x){
  parent = x;
}

int node::GetParent(){
  return parent;
}

void node::SetIndex(int x){
  index = x;
}

int node::GetIndex(){
  return index;
}

void node::SetSortedManeuvers(std::vector<int> x){
  sorted_maneuvers = x;
}

std::vector<int> node::GetSortedManeuvers(){
  return sorted_maneuvers;
}

void node::SetPosition(gazebo::math::Vector3 x){
  position = x;
}

gazebo::math::Vector3 node::GetPosition(){
  return position;
}
