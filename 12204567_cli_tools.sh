#!/bin/sh

#-------------Counfiguring environment---------------------------

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

printenv | grep -i ROS

#Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:
export ROS_DOMAIN_ID=10
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
export ROS_LOCALHOST_ONLY=1
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc


#---------------------Using turtlesim, ros2 and rqt------------------

#Install the turtlesim package for your ROS 2 distro:
sudo apt update
sudo apt install ros-humble-turtlesim

#Check that the package is installed:
ros2 pkg executables turtlesim

#To start turtlesim, enter the following command in your terminal:
ros2 run turtlesim turtlesim_node

#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key

#Open a new terminal to install rqt and its plugins:
sudo apt update
sudo apt install ~nros-humble-rqt*
rqt

#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel


#------------------Understanding nodes---------------------------

ros2 run turtlesim turtlesim_node

#ros2 node list will show you the names of all running nodes
ros2 node list
#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key
ros2 node list
#open a new terminal and run this code
#let’s reassign the name of our /turtlesim node
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
ros2 node list
#open a new terminal and run this code
ros2 node info /my_turtle


# ---------------------------Understanding Topics---------------------------

#Open a new terminal and run:
ros2 run turtlesim turtlesim_node

#Open another terminal and run:
ros2 run turtlesim turtle_teleop_key

#To run rqt_graph, open a new terminal and enter the command:
rqt_graph

#Open another terminal and run:
#To see the data being published on a topic, use:
ros2 topic list
ros2 topic list -t
ros2 topic echo /turtle1/cmd_vel
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist

# To publish data onto a topic directly from the command line using:
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#to get the turtle to keep moving, you can run:
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#you can run echo on the pose topic and recheck rqt_graph:
ros2 topic echo /turtle1/pose
# you can view the rate at which data is published using:
ros2 topic hz /turtle1/pose


#--------------------------- Understanding services-------------------------


#open a new terminal and run this code
ros2 run turtlesim turtlesim_node

#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key
ros2 service list

#open a new terminal and run this code
ros2 service type /clear
ros2 service list -t

#you can find all the Empty typed services like this
ros2 service find std_srvs/srv/Empty

#call services from the command line
ros2 interface show std_srvs/srv/Empty

# To see the request and response arguments of the /spawn service, run the command:
ros2 interface show turtlesim/srv/Spawn

#This command will clear the turtlesim window of any lines your turtle has drawn
ros2 service call /clear std_srvs/srv/Empty

#turtlesim window will update with the newly spawned turtle right away:
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"


# --------------------- Understanding parameters ----------------------

#open a new terminal and run this code
ros2 run turtlesim turtlesim_node

#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key
ros2 param list
# To find out the current value of /turtlesim’s parameter background_g:
ros2 param get /turtlesim background_g

#To change a parameter’s value at runtime
ros2 param set /turtlesim background_r 150

# To view all of a node’s current parameter values by using the command
ros2 param dump /turtlesim > turtlesim.yaml

# To load parameters from a file to a currently running node using the command:
ros2 param load /turtlesim turtlesim.yaml

#To start the same node using your saved parameter values
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml

# --------------------------- Understanding actions ---------------------------

#open a new terminal and run this code
ros2 run turtlesim turtlesim_node

#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key

#To see the list of actions a node provides
ros2 node info /turtlesim
#To identify all the actions in the ROS graph
ros2 action list
#To find /turtle1/rotate_absolute’s type, run the command:
ros2 action list -t

ros2 action info /turtle1/rotate_absolute

ros2 interface show turtlesim/action/RotateAbsolute

#To send an action goal from the command line
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
#To see the feedback of this goal, add --feedback to the ros2 action send_goal command:
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback


# --------------------------- Using rqt_console to view logs ---------------------------

#open a new terminal and run this code
ros2 run rqt_console rqt_console

#open a new terminal and run this code
ros2 run turtlesim turtlesim_node

#open a new terminal and run this code
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

# TO set the default logger level when you first run the /turtlesim node using remapping.
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN



# --------------------------- Launching nodes ---------------------------

#To launch two turtlesim nodes, open a new terminal and run the following command:
ros2 launch turtlesim multisim.launch.py

#To control the first turtlesim node (/turtlesim1), open a second terminal and run the following command to make it move clockwise:
ros2 topic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#To control the second turtlesim node (/turtlesim2), open a third terminal and run the following command to make it move counterclockwise:
ros2 topic pub /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"



# --------------------------- Recording and playing back data ---------------------------

#open a new terminal and run this code
ros2 run turtlesim turtlesim_node

#open a new terminal and run this code
ros2 run turtlesim turtle_teleop_key

mkdir bag_files
cd bag_files
ros2 topic list

#To see the data that /turtle1/cmd_vel is publishing, run the command:
ros2 topic echo /turtle1/cmd_vel

#To record the data published to a topic use the command syntax:
ros2 bag record /turtle1/cmd_vel
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

#See details about your recording by running:
ros2 bag info subset

# To play the recording
ros2 bag play subset

# To get an idea of how often position data is published, you can run the command:
ros2 topic hz /turtle1/pose





