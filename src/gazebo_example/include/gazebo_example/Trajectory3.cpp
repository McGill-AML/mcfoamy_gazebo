/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Eitan Bulka, <eitan.bulka@mail.mcgill.ca> 2018
 *
 */

#include "Trajectory3.hpp"


Trajectory::Trajectory(std::string filename_csv) {
    trajectory_number = -1;
    LoadTrajectory(filename_csv, data);
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


void Trajectory::LoadTrajectory(const std::string& filename, Eigen::MatrixXd &matrix) {
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


void Trajectory::SetNumberOfLines(const std::string& filename) {
    number_of_lines = 0;
    std::string line;
    std::ifstream myfile(filename.c_str());

    while (getline(myfile, line)) {
        ++number_of_lines;
    }

}

int Trajectory::GetNumberOfLines(){
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

double Trajectory::GetFinalTime(){
  return dt * (number_of_lines-1);
}



gazebo::math::Vector3 Trajectory::TransformPointToCameraFrame(gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, int index){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd state = GetStateAtIndex(index);
  gazebo::math::Vector3 p_k_node_Li(state(1),state(2),state(3));
  return C_cb * q.GetAsMatrix3().Inverse() * (C_i_Li * p_k_node_Li + p_node_aircraft_i);
}

double Trajectory::DistanceToObstacle(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i)
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
  
}
bool Trajectory::NoCollision(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i, gazebo::math::Vector3 p_node_i){
  // if implemented could make faster by stopping distancetoobstacle when reaching collision distance.
  double distance_to_obstacle = DistanceToObstacle(octree,q,psi_node,p_node_aircraft_i,p_node_i);
  if (distance_to_obstacle < d_max/2.0){
    return false;
  }
  else
    return true;
}

double Trajectory::DistanceToIntermediateGoal(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 intermediate_goal_i){
  return (intermediate_goal_i - End_Position(psi_node, p_node_i)).GetLength();
}

double Trajectory::YawDistanceToGoal(double psi_node, gazebo::math::Vector3 p_i, gazebo::math::Vector3 p_goal_i){
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

double Trajectory::AngleDistanceToGoal(double psi_node, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i){
  gazebo::math::Vector3 f_hat_i = End_Quaternion(psi_node).GetAsMatrix3() * gazebo::math::Vector3(1.0, 0.0, 0.0);
  return acos(f_hat_i.Dot((p_goal_i - p_initial_i).Normalize()));
}

double Trajectory::AngleDistanceToGoal2(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i){
  gazebo::math::Vector3 relative_end_position_i = End_Position(psi_node, p_node_i) - p_initial_i;
  double climb_angle = atan(-relative_end_position_i.z / sqrt(pow(relative_end_position_i.x,2.0) + pow(relative_end_position_i.y,2.0)));
  gazebo::math::Vector3 g_hat_i = gazebo::math::Quaternion(0.0, climb_angle, End_Yaw(psi_node)).GetAsMatrix3() * gazebo::math::Vector3(1.0, 0.0, 0.0);
  return acos(g_hat_i.Dot((p_goal_i - p_initial_i).Normalize()));
}

double Trajectory::DistanceToGoal(double psi_node, gazebo::math::Vector3 p_node_i, gazebo::math::Vector3 p_initial_i, gazebo::math::Vector3 p_goal_i){
  double yaw_distance = YawDistanceToGoal(psi_node, p_initial_i, p_goal_i);
  double z_distance = fabs((p_goal_i - End_Position(psi_node, p_node_i)).z);
  return yaw_distance/3.14159265 + z_distance/4.0;

}

gazebo::math::Vector3 Trajectory::End_Position(double psi_node, gazebo::math::Vector3 p_node_i){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd final_state = GetStateAtIndex(number_of_lines-1);
  gazebo::math::Vector3 p_kf_node_Li(final_state(1),final_state(2),final_state(3));
  return C_i_Li * p_kf_node_Li + p_node_i;
}

gazebo::math::Vector3 Trajectory::GetPosition(double psi_node, gazebo::math::Vector3 p_node_i, double t){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd state = GetStateAtTime(t);
  gazebo::math::Vector3 p_k_node_Li(state(1),state(2),state(3));
  return C_i_Li * p_k_node_Li + p_node_i;
}

gazebo::math::Vector3 Trajectory::GetPositionFromIndex(double psi_node, gazebo::math::Vector3 p_node_i, int k){
  gazebo::math::Matrix3 C_i_Li(cos(psi_node),-sin(psi_node),0.0,sin(psi_node),cos(psi_node),0.0,0.0,0.0,1.0);//rotate by yaw offset
  Eigen::VectorXd state = GetStateAtIndex(k);
  gazebo::math::Vector3 p_k_node_Li(state(1),state(2),state(3));
  return C_i_Li * p_k_node_Li + p_node_i;
}

double Trajectory::End_Yaw(double psi_node){
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

gazebo::math::Quaternion Trajectory::End_Quaternion(double psi_node){
  Eigen::VectorXd final_state = GetStateAtIndex(number_of_lines-1);
  gazebo::math::Quaternion q_node(0.0, 0.0, psi_node);
  gazebo::math::Quaternion q_kf_Lb(final_state(4),final_state(5),final_state(6),final_state(7));
  return q_node * q_kf_Lb;
}

gazebo::math::Quaternion Trajectory::GetQuaternion(double psi_node, double t){
  Eigen::VectorXd state = GetStateAtTime(t);
  gazebo::math::Quaternion q_psi_node(0.0, 0.0, psi_node);
  gazebo::math::Quaternion q_no_offset(state(4),state(5),state(6),state(7));
  gazebo::math::Quaternion q = q_psi_node * q_no_offset;
  return q;
}

bool Trajectory::InFieldOfView(gazebo::math::Quaternion q, double psi_node, gazebo::math::Vector3 p_node_aircraft_i){
  gazebo::math::Vector3 p_c = TransformPointToCameraFrame(q, psi_node, p_node_aircraft_i, number_of_lines-1);
  bool out = true;
  double HFOV = 85.2*3.14/180.0;//for realsense d435
  double VFOV = 58.0*3.14/180.0;//for realsense d435
  double range = 10.0; //for realsense d435
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


node::node(){

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

void node::SetSortedManeuvers(std::vector<size_t> x){
  sorted_maneuvers = x;
}

std::vector<size_t> node::GetSortedManeuvers(){
  return sorted_maneuvers;
}

void node::SetPosition(gazebo::math::Vector3 x){
  position_i = x;
}

gazebo::math::Vector3 node::GetPosition(){
  return position_i;
}

void node::SetYaw(double x){
  yaw = x;
}

double node::GetYaw(){
  return yaw;
}


void node::SetMinDistToObst(double x){
  MinDistToObst = x;
}

double node::GetMinDistToObst(){
  return MinDistToObst;
}

void node::SetInFOV(bool x){
  InFOV = x;
}

bool node::GetInFOV(){
  return InFOV;
}

void node::SetCollision(bool x){
  Collision = x;
}

bool node::GetCollision(){
  return Collision;
}

void node::SetTrajectoryToNode(int x){
  TrajectoryToNode = x;
}

int node::GetTrajectoryToNode(){
  return TrajectoryToNode;
}


TrajectoryLibrary::TrajectoryLibrary(){
  lambda = 0.0;
  C_cb = gazebo::math::Matrix3(0.0, 1.0, 0.0, -sin(lambda), 0.0, cos(lambda), cos(lambda), 0.0, sin(lambda));

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

int TrajectoryLibrary::GetNumberOfTrajectories(){
  return number_of_trajectories;
}

std::vector<int> TrajectoryLibrary::Rollout(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_initial_i,gazebo::math::Vector3 p_goal_i, std::vector<node> *nodes_out, std::vector<node> *final_nodes_out){
  std::vector<node> nodes;
  node q_start;
  q_start.SetPosition(p_initial_i);
  q_start.SetYaw(q.GetYaw());
  q_start.SetParent(-1);
  q_start.SetMinDistToObst(1000.0);
  q_start.SetInFOV(true);
  q_start.SetCollision(false);
  q_start.SetTrajectoryToNode(-1);
  q_start.SetIndex(0);
  nodes.push_back(q_start);
  
  int index;
  int number_of_nodes = 1;

  node q_curr;
  double trajectory_distance_to_obstacle;
  std::vector<int> queue;
  queue.push_back(0);
  int best_index = -1;
  double lowest_cost = 100000000.0;

  while (queue.size() > 0){
    index = queue[0];
    q_curr = nodes[index];
    queue.erase(queue.begin());
    if (nodes[index].GetInFOV() == false && nodes[index].GetCollision() == false){
      //end node for collison free sequence of trajectories
      double yaw_distance = fabs(atan2((p_goal_i - p_initial_i).y , (p_goal_i - p_initial_i).x) - nodes[index].GetYaw());
      double z_distance = fabs((p_goal_i - nodes[index].GetPosition()).z);
      if (nodes[index].GetMinDistToObst() > 10.0){
        nodes[index].SetMinDistToObst(10.0);// obstacles more than 10 meters are treated equal to 10 away
      }
      //double cost = 2.0*(yaw_distance + z_distance) - nodes[index].GetMinDistToObst();
      double h = -nodes[index].GetPosition().z;
      if (h > 5.0){
        h = 5.0;
      }
      double cost = 0.1*pow(pow(((p_goal_i - nodes[index].GetPosition() ).x),2.0) + pow(((p_goal_i - nodes[index].GetPosition() ).y),2.0),0.5) + 2.0*fabs((p_goal_i - nodes[index].GetPosition() ).z) - nodes[index].GetMinDistToObst() ;
      //double cost = z_distance + ((nodes[index].GetPosition() - p_initial_i) - ((nodes[index].GetPosition() - p_initial_i).Dot((p_goal_i - p_initial_i).Normalize())) * (p_goal_i - p_initial_i).Normalize()).GetLength() - nodes[index].GetMinDistToObst();
      if (cost < lowest_cost){
        lowest_cost = cost;
        best_index = index;
      }

    } else if(nodes[index].GetInFOV() == true && nodes[index].GetCollision() == false){
      for (int i = 0; i < GetNumberOfTrajectories(); ++i){  
        node q_new;
        trajectory_distance_to_obstacle = GetTrajectoryAtIndex(i).DistanceToObstacle(octree,q,q_curr.GetYaw(),q_curr.GetPosition() - p_initial_i,q_curr.GetPosition());
        if (q_curr.GetMinDistToObst() < trajectory_distance_to_obstacle)
          q_new.SetMinDistToObst(q_curr.GetMinDistToObst());
        else{
          q_new.SetMinDistToObst(trajectory_distance_to_obstacle);
        }
        if (q_new.GetMinDistToObst() < 1.0/2.0){//HARDCODED PLANE WINGSPAN
          q_new.SetCollision(true);
        }else{
          q_new.SetCollision(false);
        }
        q_new.SetTrajectoryToNode(i);
        q_new.SetInFOV(GetTrajectoryAtIndex(i).InFieldOfView(q,q_curr.GetYaw(),q_curr.GetPosition() - p_initial_i));
        q_new.SetParent(index);
        q_new.SetIndex(number_of_nodes);
        q_new.SetPosition(GetTrajectoryAtIndex(i).End_Position(q_curr.GetYaw(),q_curr.GetPosition()));
        q_new.SetYaw(GetTrajectoryAtIndex(i).End_Yaw(q_curr.GetYaw()));
        nodes.push_back(q_new);
        queue.push_back(q_new.GetIndex());
        ++number_of_nodes;
      }
    } else{
      //index = q_curr.GetParent();
    }
  }


  std::vector<int> final_trajectories_reverse_order;
  std::vector<int> final_trajectories;

  int i = best_index;
  
  while (i > 0){
    final_trajectories_reverse_order.push_back(nodes[i].GetTrajectoryToNode());
    i = nodes[i].GetParent();
  }
  for (int i = final_trajectories_reverse_order.size()-1; i >= 0; i--){  
    final_trajectories.push_back(final_trajectories_reverse_order[i]);
  }
  *nodes_out = nodes;
  
  std::vector<node> final_nodes;
  i = best_index;
  while (i > 0){
    final_nodes.push_back(nodes[i]);
    i = nodes[i].GetParent();
  }


  *final_nodes_out = final_nodes;

  return final_trajectories;
}





void TrajectoryLibrary::SelectTrajectories3(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, gazebo::math::Quaternion q,gazebo::math::Vector3 p_initial_i,gazebo::math::Vector3 p_goal_i, std::vector<node> *nodes_out, std::vector<node> *final_nodes_out){
  std::vector<node> nodes;
  node q_start;
  q_start.SetPosition(p_initial_i);
  q_start.SetYaw(q.GetYaw());
  q_start.SetParent(-1);
  size_t index = 0;
  size_t number_of_nodes = 1;
  q_start.SetIndex(index);






  std::vector<double> distances_to_goal;
  for (int i = 0; i < number_of_trajectories; ++i){
    distances_to_goal.push_back(GetTrajectoryAtIndex(i).DistanceToGoal(q_start.GetYaw(),q_start.GetPosition(),p_initial_i, p_goal_i));
  } 

  std::vector<size_t> sorted_distances_to_goal_indices;
  std::vector<double> sorted_distances_to_goal;
  sort(distances_to_goal,sorted_distances_to_goal,sorted_distances_to_goal_indices);
  q_start.SetSortedManeuvers(sorted_distances_to_goal_indices);
  nodes.push_back(q_start);

  std::vector<size_t> indices;

  node q_curr = q_start;
  double tolerance = 3.0;
  bool nobreak;
  node q_new;
  bool loop_on = true;
  bool ata = false;
  while (loop_on){
    indices.push_back(index);
    nobreak = true;

    for (size_t i = 0; i < q_curr.GetSortedManeuvers().size(); ++i){  
      if (GetTrajectoryAtIndex(q_curr.GetSortedManeuvers()[0]).NoCollision(octree,q,q_curr.GetYaw(),q_curr.GetPosition() - p_initial_i,q_curr.GetPosition())){
        if (GetTrajectoryAtIndex(q_curr.GetSortedManeuvers()[0]).InFieldOfView(q,q_curr.GetYaw(),q_curr.GetPosition() - p_initial_i)){
          //make new node
          if (q_curr.GetSortedManeuvers()[0] == 0){
            //ata
            ata = true;
            loop_on = false;
            break;
          }
          q_new.SetParent(index);
          q_new.SetIndex(number_of_nodes);
          q_new.SetPosition(GetTrajectoryAtIndex(q_curr.GetSortedManeuvers()[0]).End_Position(q_curr.GetYaw(),q_curr.GetPosition()));
          q_new.SetYaw(GetTrajectoryAtIndex(q_curr.GetSortedManeuvers()[0]).End_Yaw(q_curr.GetYaw()));
          distances_to_goal.clear();
          sorted_distances_to_goal.clear();
          sorted_distances_to_goal_indices.clear();
          for (int j = 0; j < number_of_trajectories; ++j){
            distances_to_goal.push_back(GetTrajectoryAtIndex(j).DistanceToGoal(q_new.GetYaw(), q_new.GetPosition(), p_initial_i, p_goal_i));
          }
          sort(distances_to_goal,sorted_distances_to_goal,sorted_distances_to_goal_indices);
          q_new.SetSortedManeuvers(sorted_distances_to_goal_indices);
          nodes.push_back(q_new);
          ++number_of_nodes;
          index = q_new.GetIndex();
          q_curr = q_new;
        }
        else{
          //this trajectory took us out of field of view so last new node
          loop_on = false;
        }
        nobreak = false;
        break; 
      }
      else{
        std::vector<size_t> temp_sorted_maneuvers = q_curr.GetSortedManeuvers();
        temp_sorted_maneuvers.erase(temp_sorted_maneuvers.begin());
        q_curr.SetSortedManeuvers(temp_sorted_maneuvers);
        nodes[index] = q_curr;
      }
    }
    if (nobreak){
      index = q_curr.GetParent();
      if (index == -1){
        break;
      }
      q_curr = nodes[index];
      std::vector<size_t> temp_sorted_maneuvers = q_curr.GetSortedManeuvers();
      temp_sorted_maneuvers.erase(temp_sorted_maneuvers.begin());
      q_curr.SetSortedManeuvers(temp_sorted_maneuvers);
      nodes[index] = q_curr;
    }



  }



  std::vector<node> final_nodes;
  int i = index;
  while (i > -1){
    final_nodes.push_back(nodes[i]);
    i = nodes[i].GetParent();
  }


  *nodes_out = nodes;
  *final_nodes_out = final_nodes;

}