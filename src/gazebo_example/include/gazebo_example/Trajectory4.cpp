/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Eitan Bulka, <eitan.bulka@mail.mcgill.ca> 2019
 *
 */

#include "Trajectory4.hpp"


TrimTrajectory::TrimTrajectory(Eigen::VectorXd trim_states, float duration, int ind) {
	psi_dot = trim_states(0);
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
}

gazebo::math::Vector3 TrimTrajectory::GetPositionAtTime(float delta_t, gazebo::math::Vector3 p, float psi){ //delta_t is time since t2
	gazebo::math::Vector3 position_at_t;

	position_at_t.x = p.x + (V_xy / psi_dot) * (sinf(psi + psi_dot * delta_t) - sinf(psi));
	position_at_t.y = p.y + (-V_xy / psi_dot) * (cosf(psi + psi_dot * delta_t) - cosf(psi));
	position_at_t.z = p.z + z_dot * delta_t;

	return position_at_t;
}


/*double TrimTrajectory::DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i)
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
        p_c = TransformPointToCameraFrame(q,psi_node,p_node_aircraft_i,i); 
        searchPoint.x = p_c[0];
        searchPoint.y = p_c[1];
        searchPoint.z = p_c[2];
        octree.nearestKSearch (searchPoint, 1, closest_point_index, closest_distance_squared);
        if (sqrt(closest_distance_squared[0]) < distance){
          distance = sqrt(closest_distance_squared[0]);
          // Don't keep searching for min distance because this trajectory is crashing anyways
          if (distance < d_max/2.0){
            break;
          }
        }
        else
        {
          i = i + 14;
          //double t_free = sqrt(closest_distance_squared[0])/max_speed;
          //i = i + int(t_free / dt); // assuming fixed time step in trajectory
        }


      }
    }
  }

  return distance;
  
} */


CollisionAvoidance::CollisionAvoidance() {
	V = 7.0;
	lambda = 0.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));
    HFOV = 85.2*3.14/180.0;//for realsense d435
  	VFOV = 58.0*3.14/180.0;//for realsense d435
  	range = 20.0; //for realsense d435

}


void CollisionAvoidance::LoadTrimTrajectories(const std::string& filename) {
	int number_of_lines = 0;
    std::string line;
    std::ifstream myfile(filename.c_str());

    while (getline(myfile, line)) {
        ++number_of_lines;
    }
    trim_trajectories.resize(number_of_lines,9); // minus 1 for header, i = number of columns

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
	float theta_l = atan2(p_final.y - p_initial.y, p_final.x - p_initial.x) - psi_initial;
	float r_xy = sqrt(powf(p_final.x - p_initial.x, 2.0) + powf(p_final.y - p_initial.y, 2.0)) / (2*sinf(theta_l));
	float L = d * theta_l / sinf(theta_l);
	float delta_t = L / V;
	float z_dot = (p_final.z - p_initial.z) / delta_t;
	float psi_dot = sqrt(powf(V, 2.0) - powf(z_dot, 2.0));



	int trajectory_index = 4;
	TrimTrajectory ret(trim_trajectories.row(trajectory_index), delta_t, trajectory_index);

	return ret; //FIX ONCE I CREATE CSV OF TRIM PRIMS BUT SHOULD BE AN ALGEBRAIC FUNCTION OF Z_DOT ROUNDED AND PSI_DOT ROUNDED

}

std::vector<gazebo::math::Vector3> CollisionAvoidance::get_final_positions_inertial(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial){
	gazebo::math::Vector3 p_c;
	gazebo::math::Vector3 p_final;
	std::vector<gazebo::math::Vector3> final_positions_inertial;
	int N = 4;
	for (int h = -N/2; h < N/2; ++h){
		for (int v = -N/2; v < N/2; ++v){
			p_c.x = range * cosf(v * VFOV/float(N)) * sinf(h * HFOV/float(N));
			p_c.y = range * sinf(v * VFOV/float(N));
			p_c.z = range * cosf(v * VFOV/float(N)) * cosf(h * HFOV/float(N));
			p_final = p_initial + q_initial.GetAsMatrix3() * C_cb.Inverse() * p_c;
			final_positions_inertial.push_back(p_final);
		}
	}
	return final_positions_inertial;
}

int CollisionAvoidance::SelectTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial){
	int best_trajectory_index;
	float best_trajectory_cost;
	int trajectory_index;	
	std::vector<gazebo::math::Vector3> final_positions_inertial = get_final_positions_inertial(p_initial, q_initial);
	for (int i = 0; i < final_positions_inertial.size(); ++i){
		TrimTrajectory trajectory_evaluating = get_trim_trajectory(p_initial, q_initial.GetYaw(), final_positions_inertial[i]);
	}




return 0;

}




