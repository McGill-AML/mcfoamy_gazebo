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
	lambda = 0.0 * 3.14/180.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));
    d_max = 2.0;
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
          t += 0.25 / V;
        }


      }
    }
  }

  return distance;
  
} 


AgileTrajectory::AgileTrajectory(std::string filename_csv) {
    trajectory_number = -1;
    LoadTrajectory(filename_csv, data);
    dt = data(1,0) - data(0,0); //assume constant delta t in trajectory
    aircraft_geometry.push_back(gazebo::math::Vector3(0.1,0.0,0.0));
    aircraft_geometry.push_back(gazebo::math::Vector3(0.0,0.5,0.0));
    aircraft_geometry.push_back(gazebo::math::Vector3(0.0,-0.5,0.0));
    aircraft_geometry.push_back(gazebo::math::Vector3(-0.7,0.0,0.0));

    d_min = 0.2;
    d_max = 2.0;
    max_speed = 15.0;

    lambda = 0.0 * 3.14/180.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));

}


void AgileTrajectory::LoadTrajectory(const std::string& filename, Eigen::MatrixXd &matrix) {
    SetNumberOfLines(filename);
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


void AgileTrajectory::SetNumberOfLines(const std::string& filename) {
    number_of_lines = 0;
    std::string line;
    std::ifstream myfile(filename.c_str());

    while (getline(myfile, line)) {
        ++number_of_lines;
    }

}

int AgileTrajectory::GetNumberOfLines(){
  return number_of_lines;
}

Eigen::VectorXd AgileTrajectory::GetStateAtIndex(int index){
  return data.row(index);
}

int AgileTrajectory::GetIndexAtTime(double time){
  int index = int(time/dt);
  if (index > number_of_lines-1){
    index = number_of_lines-1;
  }
  return index;
}

Eigen::VectorXd AgileTrajectory::GetStateAtTime(double time){
  return GetStateAtIndex(GetIndexAtTime(time));
}

double AgileTrajectory::GetFinalTime(){
  return dt * (number_of_lines-1);
}



gazebo::math::Vector3 AgileTrajectory::TransformPointToCameraFrame(gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, int index){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd state = GetStateAtIndex(index);
  gazebo::math::Vector3 p_k_node_Li(state(1),state(2),state(3));
  return C_cb * q.GetAsMatrix3().Inverse() * (C_i_Li * p_k_node_Li + p_node_aircraft_i);
}

double AgileTrajectory::DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i)
{
  pcl::PointXYZ searchPoint;
  std::vector<int> closest_point_index(1);
  std::vector<float> closest_distance_squared(1);
  double distance = 100.0;

  int i = 0;
  searchPoint.x = 0.0;
  searchPoint.y = 0.0;
  searchPoint.z = 0.0;
  
  /*while(i < number_of_lines){
    gazebo::math::Vector3 p_i = GetPositionFromIndex(psi_node, p_node_i, i);
    if (-p_i.z < distance){
      distance = -p_i.z;
    }
    i = i + 1;
  }*/
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
        }
        if (distance < d_max/2.0){
          break;
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
  
}
bool AgileTrajectory::NoCollision(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i){
  // if implemented could make faster by stopping distancetoobstacle when reaching collision distance.
  double distance_to_obstacle = DistanceToObstacle(octree,q,psi_node,p_node_aircraft_i,p_node_i);
  if (distance_to_obstacle < d_max/2.0){
    return false;
  }
  else
    return true;
}

double AgileTrajectory::DistanceToIntermediateGoal(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 intermediate_goal_i){
  return (intermediate_goal_i - End_Position(psi_node, p_node_i)).GetLength();
}

double AgileTrajectory::YawDistanceToGoal(double psi_node, gazebo::math::Vector3 p_i, gazebo::math::Vector3 p_goal_i){
  //double yaw_distance = atan2((p_goal_i - p_i).y , (p_goal_i - p_i).x) - End_Yaw(psi_node);
  double yaw_distance = atan2((p_goal_i - p_i).y , (p_goal_i - p_i).x) - End_Quaternion(psi_node).GetYaw();

  if (yaw_distance > 3.14159265){
    yaw_distance = yaw_distance - 2.0 * 3.14159265;
  }
  if (yaw_distance < -3.14159265){
    yaw_distance = yaw_distance + 2.0 * 3.14159265;
  }

  return fabs(yaw_distance);
  
}

double AgileTrajectory::AngleDistanceToGoal(double psi_node, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i){
  gazebo::math::Vector3 f_hat_i = End_Quaternion(psi_node).GetAsMatrix3() * gazebo::math::Vector3(1.0, 0.0, 0.0);
  return acos(f_hat_i.Dot((p_goal_i - p_initial_i).Normalize()));
}

double AgileTrajectory::AngleDistanceToGoal2(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i){
  gazebo::math::Vector3 relative_end_position_i = End_Position(psi_node, p_node_i) - p_initial_i;
  double climb_angle = atan(-relative_end_position_i.z / sqrt(pow(relative_end_position_i.x,2.0) + pow(relative_end_position_i.y,2.0)));
  gazebo::math::Vector3 g_hat_i = gazebo::math::Quaternion(0.0, climb_angle, End_Yaw(psi_node)).GetAsMatrix3() * gazebo::math::Vector3(1.0, 0.0, 0.0);
  return acos(g_hat_i.Dot((p_goal_i - p_initial_i).Normalize()));
}

double AgileTrajectory::DistanceToGoal(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i){
  double yaw_distance = YawDistanceToGoal(psi_node, p_initial_i, p_goal_i);
  double z_distance = fabs((p_goal_i - End_Position(psi_node, p_node_i)).z);
  return yaw_distance/3.14159265 + z_distance/4.0;

}

gazebo::math::Vector3 AgileTrajectory::End_Position(double psi_node, gazebo::math::Vector3 p_node_i){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd final_state = GetStateAtIndex(number_of_lines-1);
  gazebo::math::Vector3 p_kf_node_Li(final_state(1),final_state(2),final_state(3));
  return C_i_Li * p_kf_node_Li + p_node_i;
}

gazebo::math::Vector3 AgileTrajectory::GetPosition(double psi_node, gazebo::math::Vector3 p_node_i, double t){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd state = GetStateAtTime(t);
  gazebo::math::Vector3 p_k_node_Li(state(1),state(2),state(3));
  return C_i_Li * p_k_node_Li + p_node_i;
}

gazebo::math::Vector3 AgileTrajectory::GetPositionFromIndex(double psi_node, gazebo::math::Vector3 p_node_i, int k){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd state = GetStateAtIndex(k);
  gazebo::math::Vector3 p_k_node_Li(state(1),state(2),state(3));
  return C_i_Li * p_k_node_Li + p_node_i;
}

double AgileTrajectory::End_Yaw(double psi_node){
  Eigen::VectorXd final_state = GetStateAtIndex(number_of_lines-1);
  gazebo::math::Quaternion q_final(final_state(4),final_state(5),final_state(6),final_state(7));
  double end_yaw = psi_node + q_final.GetYaw();
  if (end_yaw > 3.14159265){
    end_yaw = end_yaw - 2.0 * 3.14159265;
  }
  if (end_yaw < -3.14159265){
    end_yaw = end_yaw + 2.0 * 3.14159265;
  }
  return end_yaw;
}

gazebo::math::Quaternion AgileTrajectory::End_Quaternion(double psi_node){
  Eigen::VectorXd final_state = GetStateAtIndex(number_of_lines-1);
  gazebo::math::Quaternion q_node(0.0, 0.0, psi_node);
  gazebo::math::Quaternion q_kf_Lb(final_state(4),final_state(5),final_state(6),final_state(7));
  return q_node * q_kf_Lb;
}

gazebo::math::Quaternion AgileTrajectory::GetQuaternion(double psi_node, double t){
  Eigen::VectorXd state = GetStateAtTime(t);
  gazebo::math::Quaternion q_psi_node(0.0, 0.0, psi_node);
  gazebo::math::Quaternion q_no_offset(state(4),state(5),state(6),state(7));
  gazebo::math::Quaternion q = q_psi_node * q_no_offset;
  return q;
}

gazebo::math::Vector3 AgileTrajectory::GetVelocity(double t){
	Eigen::VectorXd reference_state = GetStateAtTime(t);
  	return gazebo::math::Vector3(reference_state[8],reference_state[9],reference_state[10]);
 }


gazebo::math::Vector3 AgileTrajectory::GetAngularVelocity(double t){
	Eigen::VectorXd reference_state = GetStateAtTime(t);
  	return gazebo::math::Vector3(reference_state[11],reference_state[12],reference_state[13]);  
}

bool AgileTrajectory::InFieldOfView(gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i){
  gazebo::math::Vector3 p_c = TransformPointToCameraFrame(q, psi_node, p_node_aircraft_i, number_of_lines-1);
  bool out = true;
  double HFOV = 85.2*3.14/180.0;//for realsense d435
  double VFOV = 58.0*3.14/180.0;//for realsense d435
  double range = 20.0; //for realsense d435
  if (fabs(atan2(p_c.x,p_c.z)) > HFOV/2.0){
    out = false;
  }

  if (fabs(atan2(p_c.y,p_c.z)) > VFOV/2.0){
    out = false;
  }
  if (p_c.GetLength() > range){
    out = false;
  }
  return out;
}


CollisionAvoidance::CollisionAvoidance() {
	//V = 7.0;
	lambda = 0.0 * 3.14/180.0;
    //C_cb = gazebo::math::Matrix3(0.0,-sin(lambda),cos(lambda),1.0,0.0,0.0,0.0,cos(lambda),sin(lambda));
    C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));
    HFOV = 65.0*PI/180.0;//for realsense d435
  	VFOV = 58.0*PI/180.0;//for realsense d435
  	range = 20.0; //for realsense d435
  	d_max = 2.0;
  	ata_count = 0;
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
    V = powf(powf(trim_trajectories.row(0)(4),2.0) + powf(trim_trajectories.row(0)(5),2.0) + powf(trim_trajectories.row(0)(6),2.0),0.5);
    /*max_turn_rate = 110;
    if (V < 14.0)
    {
      max_turn_rate = 30;//V =13
    }
    if (V < 12.0)
    {
      max_turn_rate = 70;//V =11
    }
    if (V < 10.0)
    {
      max_turn_rate = 90;//V =9
    }
    if (V < 8.0)
    {
      max_turn_rate = 110;//V =7
    }*/
}

TrimTrajectory CollisionAvoidance::get_trim_trajectory(gazebo::math::Vector3 p_initial, float psi_initial, gazebo::math::Vector3 p_final){
	float d = (p_final - p_initial).GetLength();
	float theta_l = atan2f(p_final.y - p_initial.y, p_final.x - p_initial.x) - psi_initial;
	theta_l = fmod(theta_l, 2.0 * PI);
	if (theta_l > PI){theta_l = -2.0 * PI + theta_l;}
	else if (theta_l < -PI) {theta_l = 2.0 * PI + theta_l;}

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
		L = d * theta_l / sinf(theta_l); if (L > 2.0 * range){L = 2.0 * range;}
		delta_t = L / V;
		z_dot = (p_final.z - p_initial.z) / delta_t;
		psi_dot = sqrt(powf(V, 2.0) - powf(z_dot, 2.0)) / r_xy;
	}
	if (delta_t > 10){printf("%f\n", delta_t);printf("%f\n", L);}




	int z_dot_rounded = round(z_dot);
	int psi_dot_deg_rounded = round((psi_dot * 180.0 / PI) / 10.0) * 10;
	//int trajectory_index = -1;
  int trajectory_index = 57;

  //if (z_dot_rounded > 2){z_dot_rounded = 2;}
  //if (z_dot_rounded < -2){z_dot_rounded = -2;}
  //if (psi_dot_deg_rounded > 50){psi_dot_deg_rounded = 50;}
  //if (psi_dot_deg_rounded < -50){psi_dot_deg_rounded = -50;}


	if (z_dot_rounded >= -2 && z_dot_rounded <= 2 && psi_dot_deg_rounded >= -110 && psi_dot_deg_rounded <= 110){
		trajectory_index = (psi_dot_deg_rounded + 110) * 5 / 10 + (z_dot_rounded + 2) / 1;
    TrimTrajectory checking_psi_dot(trim_trajectories.row(trajectory_index), delta_t, trajectory_index);
    if (abs(checking_psi_dot.psi_dot - psi_dot)*180.0/3.14 > 10){
      //trajectory_index = -1;
    }

	}

	int trajectory_index2 = 0;
	if (trajectory_index != -1){
		trajectory_index2 = trajectory_index;
	}

	TrimTrajectory ret(trim_trajectories.row(trajectory_index2), delta_t, trajectory_index);

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

  for (int h = -N/2; h <= N/2; h += 4){
    for (int v = -N/2; v <= N/2; ++v){
      p_c.x = (12.0) * cosf(v * VFOV/float(N)) * sinf(h * HFOV/float(N));
      p_c.y = (12.0) * sinf(v * VFOV/float(N));
      p_c.z = (12.0) * cosf(v * VFOV/float(N)) * cosf(h * HFOV/float(N));
      p_final = p_initial + q_initial.GetAsMatrix3() * C_cb.Inverse() * p_c; 
      final_positions_inertial.push_back(p_final); 
    }
  }  

  for (int h = -1; h <= 1; ++h){
    for (int v = -N/2; v <= N/2; v += 4){
      p_c.x = (12.0) * cosf(v * VFOV/float(N)) * sinf(h * HFOV/float(N));
      p_c.y = (12.0) * sinf(v * VFOV/float(N));
      p_c.z = (12.0) * cosf(v * VFOV/float(N)) * cosf(h * HFOV/float(N));
      p_final = p_initial + q_initial.GetAsMatrix3() * C_cb.Inverse() * p_c; 
      final_positions_inertial.push_back(p_final); 
    }
  }  
	return final_positions_inertial;
}

int CollisionAvoidance::SelectTrimTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Vector3 p_goal, std::vector<TrimTrajectory> *trajectories_out, std::vector<gazebo::math::Vector3> *final_positions_inertial_out, std::vector<int> trajectory_packet_prev){
	int best_trajectory_index = -1;
	float lowest_trajectory_cost = 9999999999;
	float cost;
	float distance_to_obstacle;

	std::vector<gazebo::math::Vector3> final_positions_inertial = get_final_positions_inertial(p_initial, q_initial);
	*final_positions_inertial_out = final_positions_inertial;

	std::vector<TrimTrajectory> trajectories;

	for (int i = 0; i < final_positions_inertial.size(); ++i){ 
		TrimTrajectory trajectory_evaluating = get_trim_trajectory(p_initial, q_initial.GetYaw(), final_positions_inertial[i]);//printf("index %i\n", i); printf("traj_index %i\n", trajectory_evaluating.index);
		if (trajectory_evaluating.index != -1){
			trajectories.push_back(trajectory_evaluating); 
			distance_to_obstacle = trajectory_evaluating.DistanceToObstacle(octree, q_initial, p_initial, trajectory_evaluating.delta_t);
			if (distance_to_obstacle >= d_max/2.0){
          double yaw_distance = atan2((p_goal - p_initial).y , (p_goal - p_initial).x) - trajectory_evaluating.GetQuaternionAtTime(trajectory_evaluating.delta_t, q_initial.GetYaw()).GetYaw();
          if (yaw_distance > 3.14159265){
            yaw_distance = yaw_distance - 2.0 * 3.14159265;
          }
          if (yaw_distance < -3.14159265){
            yaw_distance = yaw_distance + 2.0 * 3.14159265;
          }

          yaw_distance = fabs(yaw_distance);
  
				cost = 15.0 * yaw_distance + 2.0 * fabs((p_goal - final_positions_inertial[i]).z) - 2.0 * distance_to_obstacle;
				/*if ((p_goal - p_initial).GetLength() > range){
          cost += 0.1 * sqrt(powf((p_goal - final_positions_inertial[i]).x, 2.0) + powf((p_goal - final_positions_inertial[i]).y, 2.0));
        }
        else{
          cost += 10 * yaw_distance;
        }*/
        if (trajectory_packet_prev[0] == 0){
					float psi_dot_deg_prev = trim_trajectories.row(trajectory_packet_prev[1])(0);
					float z_dot_prev = trim_trajectories.row(trajectory_packet_prev[1])(1);
					cost += .03 * fabs(trajectory_evaluating.psi_dot_deg - psi_dot_deg_prev) + .00 * fabs(trajectory_evaluating.z_dot - z_dot_prev);
				}
				if (cost < lowest_trajectory_cost){
					lowest_trajectory_cost = cost;
					best_trajectory_index = trajectory_evaluating.index;
				}
			}
		}	
	}

*trajectories_out = trajectories;	//printf("best trajectory index %i\n", best_trajectory_index);
return best_trajectory_index; 
}

void CollisionAvoidance::LoadAgileLibrary(std::vector<std::string> filenames){
  number_of_agile_trajectories = filenames.size();
  for (int i = 0; i < number_of_agile_trajectories; ++i){
    agile_trajectory_library.push_back(AgileTrajectory(filenames[i]));
  }
}

AgileTrajectory CollisionAvoidance::GetAgileTrajectoryAtIndex(int index){
  return agile_trajectory_library[index];
}

int CollisionAvoidance::GetNumberOfAgileTrajectories(){
  return number_of_agile_trajectories;
}

std::vector<int> CollisionAvoidance::SelectTrajectory(gazebo::math::Vector3 p_initial, gazebo::math::Quaternion q_initial, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Vector3 p_goal, std::vector<TrimTrajectory> *trajectories_out, std::vector<gazebo::math::Vector3> *final_positions_inertial_out, std::vector<int> trajectory_packet_prev, bool restart, double time){
	std::vector<int> ret; //first index is trim/agile, second is trajectory number
	if (restart){
		mode = 2;
		mission_start_time = time;
	}

	if (mode == 0){
		if (restart){
			//get in to initial hover
			ret.push_back(1); //agile maneuver
			ret.push_back(2); //C2H
		}else{
			ret.push_back(2); //HOVER once already cruised to hover
			ret.push_back(99); // does nothing
		}
		if (time - mission_start_time > 10){
			mode = 1; //after 10 seconds of hovering, go to cruise
		}

	}
	else if (mode == 1){
		//hover to cruise
		ret.push_back(1); //agile maneuver
		ret.push_back(1); //H2C
		printf("%s\n", "Hover to Cruise");
		//only send hover to cruise once, so now enter avoidance mode
		mode = 2;
	}
	else if (mode == 2){
		//avoidance
		int trim_trajectory = SelectTrimTrajectory(p_initial, q_initial, octree, p_goal, trajectories_out, final_positions_inertial_out, trajectory_packet_prev);
		if (trim_trajectory == -1){
			//no trajectories found enter ATA
			ret.push_back(1); //agile maneuver
			//ret.push_back(0); //ATA
			//ata_count += 1;
			//printf("%i", ata_count);
			//printf("%s\n", "Fucked: Crash unless ATA");

      ret.push_back(2); //Hover
      printf("%s\n", "Fucked: Hover");
      mode = 4;

		}
		else{
			ret.push_back(0); //trim maneuver
			ret.push_back(trim_trajectory); //type of trim primitivea
		}
		if ((p_goal - p_initial).GetLength() < 5.0){
			//if inside 5 meters to goal, enter hover
			mode = 3;
		}
	}
	else if (mode == 3){
		//cruise to hover
		ret.push_back(1); //agile maneuver
		ret.push_back(2); //C2H
		printf("%s\n", "Found Goal, Cruise to Hover");
		// only cruise to hover once
		mode = 4;
	}
	else if (mode == 4){
		//end_hover
		ret.push_back(2); //HOVER 
		ret.push_back(99);
	}

	return ret;
}