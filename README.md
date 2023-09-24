# SmartMobilityEngineeringLab
#Beginner: CLI Tools

## Configuring environment:
• ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently.

• Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (or “distros”, e.g. Dashing and Eloquent) on the same computer and switching between them.

• This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.

• Prerequisites
• Before starting these tutorials, install ROS 2 by following the instructions on the ROS 2 Installation page.

## Using turtlesim, ros2, and rqt:
• Turtlesim is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level to give you an idea of what you will do with a real robot or a robot simulation later on.

• The ros2 tool is how the user manages, introspects, and interacts with a ROS system. It supports multiple commands that target different aspects of the system and its operation. One might use it to start a node, set a parameter, listen to a topic, and many more. The ros2 tool is part of the core ROS 2 installation.

• rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.

To start turtlesim, enter the following command in your terminal:

ros2 run turtlesim turtlesim_node
The simulator window should appear, with a random turtle in the center

<img src="images/Screenshot from 2023-09-19 11-21-51.png">

Now you will run a new node to control the turtle in the first node:

ros2 run turtlesim turtle_teleop_key

<img src="images/Screenshot from 2023-09-19 11-25-45.png">

To run rqt:
rqt
Let’s use rqt to call the /spawn service. You can guess from its name that /spawn will create another turtle in the turtlesim window.
Give the new turtle a unique name, like turtle2, by double-clicking between the empty single quotes in the Expression column. You can see that this expression corresponds to the value of name and is of type string.

<img src="images/Screenshot from 2023-09-20 09-24-28.png">

The values for r, g and b, which are between 0 and 255, set the color of the pen turtle1 draws with, and width sets the thickness of the line.

<img src="images/Screenshot from 2023-09-20 09-27-14.png">

You need a second teleop node in order to control turtle2. However, if you try to run the same command as before, you will notice that this one also controls turtle1. The way to change this behavior is by remapping the cmd_vel topic.

<img src="images/Screenshot from 2023-09-20 09-32-55.png">

Now, you can move turtle2 when this terminal is active, and turtle1 when the other terminal running turtle_teleop_key is active.

<img src="images/Screenshot from 2023-09-20 09-33-01.png">



## Understanding nodes:
• 
## Understanding topics:
• 
## Understanding services:
• 
## Understanding parameters:
• 
## Understanding actions:
• 
## Using rqt_console to view logs:
• 
## Launching nodes:
• 
## Recording and playing back data:
• 
## Beginner: Client libraries:
• 

<img src="images/Screenshot from 2023-09-19 11-21-51.png">
<img src="images/Screenshot from 2023-09-19 11-23-31.png">
<img src="images/Screenshot from 2023-09-19 11-25-45.png">
<img src="images/Screenshot from 2023-09-20 09-24-28.png">
<img src="images/Screenshot from 2023-09-20 09-24-36.png">
<img src="images/Screenshot from 2023-09-20 09-26-58.png">
<img src="images/Screenshot from 2023-09-20 09-27-14.png">
<img src="images/Screenshot from 2023-09-20 09-32-55.png">
<img src="images/Screenshot from 2023-09-20 09-33-01.png">
<img src="images/Screenshot from 2023-09-20 10-20-32.png">
<img src="images/Screenshot from 2023-09-20 10-21-02.png">
<img src="images/Screenshot from 2023-09-20 10-21-35.png">
<img src="images/Screenshot from 2023-09-23 19-27-28.png">
<img src="images/Screenshot from 2023-09-23 19-41-28.png">
<img src="images/Screenshot from 2023-09-23 19-42-10.png">
<img src="images/Screenshot from 2023-09-23 19-47-41.png">
<img src="images/Screenshot from 2023-09-23 19-52-09.png">
<img src="images/Screenshot from 2023-09-23 19-52-50.png">
<img src="images/Screenshot from 2023-09-24 21-24-49.png">
<img src="images/Screenshot from 2023-09-24 21-28-44.png">
<img src="images/Screenshot from 2023-09-24 21-30-28.png">
<img src="images/Screenshot from 2023-09-24 21-33-31.png">
<img src="images/Screenshot from 2023-09-24 21-42-37.png">
<img src="images/Screenshot from 2023-09-24 21-42-47.png">
<img src="images/Screenshot from 2023-09-24 21-42-52.png">
<img src="images/Screenshot from 2023-09-24 21-45-31.png">
<img src="images/Screenshot from 2023-09-24 21-48-22.png">


#Beginner: Client Libraries:

## Using colcon to build packages:
• 
## Creating a workspace:
• 
## Creating a package:
• 
## Writing a simple publisher and subscriber (C++):
• 
## Writing a simple publisher and subscriber (Python):
• 
## Writing a simple service and client (C++):
• 
## Writing a simple service and client (Python):
• 
## Creating custom msg and srv files:
• 
## Implementing custom interfaces:
• 
## Using parameters in a class (C++):
• 
## Using parameters in a class (Python):
• 
## Using ros2doctor to identify issues:
• 
## Creating and using plugins (C++):
• 



