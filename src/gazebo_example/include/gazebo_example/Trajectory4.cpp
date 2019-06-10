/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Eitan Bulka, <eitan.bulka@mail.mcgill.ca> 2019
 *
 */

#include "Trajectory4.hpp"


TrimTrajectory::TrimTrajectory(Eigen::VectorXd trim_states, float duration, int ind) {
	psi_dot_deg = trim_states(0); //in degrees!!!!
	psi_dot = psi_dot_deg * PI / 180.0; 
	z_dot = trim_states(1);
	phi = trim_states(2);
	theta = trim_states(3);
	u = trim_states(4);
	v = trim_states(5);
	w = trim_states(6);
	p = trim_states(7);
	q = trim_states(8);
	r = trim_states(9);
	V = sqrt(powf(u,2.0) + powf(v,2.0) + powf(w,2.0));
	V_xy = sqrt(powf(V,2.0) - powf(z_dot, 2.0));
	delta_t = duration;
	index = ind;
	lambda = 0.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));
    d_max = 1.0;
}

gazebo::math::Vector3 TrimTrajectory::GetPositionAtTime(float t, gazebo::math::Vector3 p, float psi){ //delta_t is time since t2
	gazebo::math::Vector3 position_at_t;
	if (psi_dot == 0){ //straight		
		position_at_t.x = p.x + V_xy * t * cosf(psi);
		position_at_t.y = p.y + V_xy * t * sinf(psi);
		position_at_t.z = p.z + z_dot * t;
	}else{ //turning
		position_at_t.x = p.x + (V_xy / psi_dot) * (sinf(psi + psi_dot * t) - sinf(psi));
		position_at_t.y = p.y + (-V_xy / psi_dot) * (cosf(psi + psi_dot * t) - cosf(psi));
		position_at_t.z = p.z + z_dot * t;
	}


	return position_at_t;
}

gazebo::math::Quaternion TrimTrajectory::GetQuaternionAtTime(float t, float psi){
	return gazebo::math::Quaternion(phi,theta, psi + psi_dot * t);
}

gazebo::math::Vector3 TrimTrajectory::GetVelocity(){
	return gazebo::math::Vector3(u,v,w);
}

gazebo::math::Vector3 TrimTrajectory::GetAngularVelocity(){
	return gazebo::math::Vector3(p,q,r);
}


float TrimTrajectory::DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q_initial, gazebo::math::Vector3 p_initial, float delta_t)
{
  pcl::PointXYZ searchPoint;
  std::vector<int> closest_point_index(1);
  std::vector<float> closest_distance_squared(1);
  float distance = 10.0;

  float t = 0.0;
  searchPoint.x = 0.0;
  searchPoint.y = 0.0;
  searchPoint.z = 0.0;
  
  if (octree.getLeafCount() > 0){
    if (octree.nearestKSearch (searchPoint, 1, closest_point_index, closest_distance_squared) > 0){
      while(t < delta_t){ 
        gazebo::math::Vector3 p_c = C_cb * q_initial.GetAsMatrix3().Inverse() * (GetPositionAtTime(t, p_initial, q_initial.GetYaw()) - p_initial);
        searchPoint.x = p_c[0];
        searchPoint.y = p_c[1];
        searchPoint.z = p_c[2];
        octree.nearestKSearch (searchPoint, 1, closest_point_index, closest_distance_squared);
        if (sqrt(closest_distance_squared[0]) < distance){
          distance = sqrt(closest_distance_squared[0]);
      	}
        // Don't keep searching for min distance because this trajectory is crashing anyways
        if (distance < d_max/2.0){
            break;
        }else{
          t += 0.86 / V;
        }


      }
    }
  }

  return distance;
  
} 


CollisionAvoidance::CollisionAvoidance() {
	V = 7.0;
	lambda = 0.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));
    HFOV = 85.2*PI/180.0;//for realsense d435
  	VFOV = 58.0*PI/180.0;//for realsense d435
  	range = 10.0; //for realsense d435
  	d_max = 1.0;
}


void CollisionAvoidance::LoadTrimTrajectories(const std::string& filename) {
	int number_of_lines = 0;
    std::string line;
    std::ifstream myfile(filename.c_str());

    while (getline(myfile, line)) {
        ++number_of_lines;
    }
    trim_trajectories.resize(number_of_lines,10); // minus 1 for header, i = number of columns

    int i =  0;
    int row_num = 0;

    CsvParser *csvparser = CsvParser_new(filename.c_str(), ",", 0);
    CsvRow *row;

    while ((row = CsvParser_getRow(csvparser)) ) {
      //printf("==NEW LINE==\n");
        const char **rowFields = CsvParser_getFields(row);
        for (i = 0 ; i < CsvParser_getNumFields(row) ; i++) {
            //printf("FIELD: %s\n", rowFields[i]);
            trim_trajectories(row_num,i) = atof(rowFields[i]);

        }
    //printf("\n");
        CsvParser_destroy_row(row);
        row_num ++;
    }
    CsvParser_destroy(csvparser);

}

TrimTrajectory CollisionAvoidance::get_trim_trajectory(gazebo::math::Vector3 p_initial, float psi_initial, gazebo::math::Vector3 p_final){
	float d = (p_final - p_initial).GetLength();
	float theta_l = atan2f(p_final.y - p_initial.y, p_final.x - p_initial.x) - psi_initial;
	theta_l = fmod(theta_l, 2 * PI);
	if (theta_l > PI){theta_l = -2 * PI + theta_l;}
	else if (theta_l < -PI) {theta_l = 2 * PI + theta_l;}

	float r_xy;
	float L;
	float psi_dot;
	float delta_t;
	float z_dot;

	if (fabs(theta_l) < 0.001){// next position is directly in front of current, i.e. curve has infinite curvature (r_xy tends to inf)
		L = d;
		delta_t = L / V;
		z_dot = (p_final.z - p_initial.z) / delta_t;
		psi_dot = 0.0;
	}
	else {
		r_xy = sqrt(powf(p_final.x - p_initial.x, 2.0) + powf(p_final.y - p_initial.y, 2.0)) / (2.0*sinf(theta_l));
		L = d * theta_l / sinf(theta_l);
		delta_t = L / V;
		z_dot = (p_final.z - p_initial.z) / delta_t;
		psi_dot = sqrt(powf(V, 2.0) - powf(z_dot, 2.0)) / r_xy;
	}




	int z_dot_rounded = round(z_dot);
	int psi_dot_deg_rounded = round((psi_dot * 180.0 / PI) / 10.0) * 10;

	if (z_dot_rounded > 2){z_dot_rounded = 2;}
	if (z_dot_rounded < -2){z_dot_rounded = -2;}

	if (psi_dot_deg_rounded > 110){psi_dot_deg_rounded = 110;}
	if (psi_dot_deg_rounded < -110){psi_dot_deg_rounded = -110;}

	//printf("zdot %f\n", z_dot); 	printf("zdot rounded %i\n", z_dot_rounded); printf("psi dot%f\n", psi_dot*180.0/PI); printf("psi dot round%i\n", psi_dot_deg_rounded);
	



	int trajectory_index = (psi_dot_deg_rounded + 110) * 5 / 10 + (z_dot_rounded + 2) / 1;
	TrimTrajectory ret(trim_trajectories.row(trajectory_index), delta_t, trajectory_index);

	return ret; 

}

std::vector<gazebo::math::Vector3> CollisionAvoidance::get_final_positions_inertial(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial){
	gazebo::math::Vector3 p_c;
	gazebo::math::Vector3 p_final;
	std::vector<gazebo::math::Vector3> final_positions_inertial; 
	int N = 4;
	for (int h = -N/2; h <= N/2; ++h){
		for (int v = -N/2; v <= N/2; ++v){
			p_c.x = range * cosf(v * VFOV/float(N)) * sinf(h * HFOV/float(N));
			p_c.y = range * sinf(v * VFOV/float(N));
			p_c.z = range * cosf(v * VFOV/float(N)) * cosf(h * HFOV/float(N));
			p_final = p_initial + q_initial.GetAsMatrix3() * C_cb.Inverse() * p_c; 
			final_positions_inertial.push_back(p_final); 
		}
	} 
	return final_positions_inertial;
}

int CollisionAvoidance::SelectTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Vector3 p_goal, std::vector<TrimTrajectory> *trajectories_out, std::vector<gazebo::math::Vector3> *final_positions_inertial_out){
	int best_trajectory_index = -1;
	float lowest_trajectory_cost = 9999999999;
	float cost;
	float distance_to_obstacle;

	std::vector<gazebo::math::Vector3> final_positions_inertial = get_final_positions_inertial(p_initial, q_initial);
	*final_positions_inertial_out = final_positions_inertial;

	std::vector<TrimTrajectory> trajectories;

	for (int i = 0; i < final_positions_inertial.size(); ++i){ 
		TrimTrajectory trajectory_evaluating = get_trim_trajectory(p_initial, q_initial.GetYaw(), final_positions_inertial[i]);//printf("index %i\n", i); printf("traj_index %i\n", trajectory_evaluating.index);
		trajectories.push_back(trajectory_evaluating); 
		distance_to_obstacle = trajectory_evaluating.DistanceToObstacle(octree, q_initial, p_initial, trajectory_evaluating.delta_t);
		if (distance_to_obstacle >= d_max/2.0){
			//uses final_positions_inertial which is wrong if primitive exceeds library bounds and is truncated
			cost = 0.1 * sqrt(powf((p_goal - final_positions_inertial[i]).x, 2.0) + powf((p_goal - final_positions_inertial[i]).y, 2.0)) + fabs((p_goal - final_positions_inertial[i]).z) - 2.0 * distance_to_obstacle;
			if (cost < lowest_trajectory_cost){
				lowest_trajectory_cost = cost;
				best_trajectory_index = trajectory_evaluating.index;
			}
		}
	}

*trajectories_out = trajectories;	//printf("best trajectory index %i\n", best_trajectory_index);
return best_trajectory_index; 
}




