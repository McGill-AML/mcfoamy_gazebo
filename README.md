# Gazebo Example

## Requirements
* ROS Kinetic full desktop version [installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Gazebo for ROS:
    * `sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control`

## Getting Started
* Create a ROS workspace:
    * `mkdir ~/WORKSPACE_NAME
* Git clone this repository in it
* Build the workspace:
    * catkin_make (optional -j4 or -j8 to make it faster; the number corresponds to the number of cores on your processor)
* Finish setting your environment by typing (update your environment variables with the right paths):
    * source setup.bash
* Run the launch files examples by typing:
    * roslaunch gazebo_example LAUNCH_FILE_NAME

## Example launch files:
* launch_example.launch

## ROS node
### controller_node
#### Subscribers
* ~pose (geometry_msgs::Pose) : pose of the robot
* ~twist (geometry_msgs::Twist) : twist of the robot 
#### Publishers
* ~external_wrench (geometry_msgs::Wrench) : 
#### Services
* ~start_controller (std_srvs::Trigger)


