#!/bin/bash
gnome-terminal -e "roslaunch gazebo_example launch_icuas19.launch" 
sleep 10
gnome-terminal -e "roslaunch gazebo_example launch_rv.launch"
gnome-terminal -e "rosservice call /gazebo/set_model_state '{model_state: { model_name: plane_xacro, pose: { position: { x: 0, y: 0 ,z: 15 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: {x: 5.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'"
gnome-terminal -e "rosservice call /start_motionplan"
gnome-terminal -e "rosservice call /start_controller"