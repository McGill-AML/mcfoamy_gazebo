## mcfoamy_gazebo

This repository contains the simulation of an agile fixed-wing aircraft avoiding obstacles. The control and obstacle avoidance algorithms are presented in Eitan Bulka's PhD thesis titled "Control and Obstacle Avoidance for Agile Fixed-Wing Aircraft".

The package uses 3 ROS nodes. The gazebo-plane.cpp node executes the aircraft dynamics, the motionplan-node.cpp node executes the obstacle avoidance algorithm, and the controller-node.cpp node executes the control algorithm.

To start simulation, new_norecord.launch starts the gazebo simulation with all nodes running, and starts control and obstacle avoidance service calls. This will not log any data. To log data, use new.launch. For extra visualization, launch-rv.launch starts RVIZ and shows the pointcloud and planning steps.
	
