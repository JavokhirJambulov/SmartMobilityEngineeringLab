# SmartMobilityEngineeringLab
ROS2 Tutorials

1. [Beginner: CLI Tools](#Beginner:-CLI-Tools)
  1. [Configuring environment]()
  2. [Using turtlesim, ros2, and rqt]()
  3. [Understanding nodes]()
  4. [Understanding topics]()
  5. [Understanding services]()
  6. [Understanding parameters]()
  7. [Understanding actions]()
  8. [Using rqt_console to view logs]()
  9. [Launching nodes]()
  10. [Recording and playing back data]()



2. # Beginner: Client libraries:
   1. [Using Colcon to Build Packages](#Using-colcon-to-build-packages:)
   2. [Creating a Workspace](#chapter-2-creating-a-workspace)
   3. [Creating a Package](#chapter-3-creating-a-package)
   4. [Writing a Simple Publisher and Subscriber (C++)](#chapter-4-writing-a-simple-publisher-and-subscriber-c)
   5. [Writing a Simple Publisher and Subscriber (Python)](#chapter-5-writing-a-simple-publisher-and-subscriber-python)
   6. [Writing a Simple Service and Client (C++)](#chapter-6-writing-a-simple-service-and-client-c)
   7. [Writing a Simple Service and Client (Python)](#chapter-7-writing-a-simple-service-and-client-python)
   8. [Creating Custom Msg and Srv Files](#chapter-8-creating-custom-msg-and-srv-files)
   9. [Implementing Custom Interfaces](#chapter-9-implementing-custom-interfaces)
   10. [Using Parameters in a Class (C++)](#chapter-10-using-parameters-in-a-class-c)
   11. [Using Parameters in a Class (Python)](#chapter-11-using-parameters-in-a-class-python)
   12. [Using ros2doctor to Identify Issues](#chapter-12-using-ros2doctor-to-identify-issues)
   13. [Chapter 13 - Creating and Using Plugins (C++)](#chapter-13-Creating-and-Using-Plugins-(C++_))


#Beginner: CLI Tools:

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

• To start turtlesim, enter the following command in your terminal:

ros2 run turtlesim turtlesim_node
• The simulator window should appear, with a random turtle in the center

<img src="images/Screenshot from 2023-09-19 11-21-51.png">

• Now you will run a new node to control the turtle in the first node:

ros2 run turtlesim turtle_teleop_key

<img src="images/Screenshot from 2023-09-19 11-25-45.png">

• To run rqt:
rqt
• Let’s use rqt to call the /spawn service. You can guess from its name that /spawn will create another turtle in the turtlesim window.
• Give the new turtle a unique name, like turtle2, by double-clicking between the empty single quotes in the Expression column. You can see that this expression corresponds to the value of name and is of type string.

<img src="images/Screenshot from 2023-09-20 09-24-28.png">

• The values for r, g and b, which are between 0 and 255, set the color of the pen turtle1 draws with, and width sets the thickness of the line.

<img src="images/Screenshot from 2023-09-20 09-27-14.png">

• You need a second teleop node in order to control turtle2. However, if you try to run the same command as before, you will notice that this one also controls turtle1. The way to change this behavior is by remapping the cmd_vel topic.

<img src="images/Screenshot from 2023-09-20 09-32-55.png">

• Now, you can move turtle2 when this terminal is active, and turtle1 when the other terminal running turtle_teleop_key is active.

<img src="images/Screenshot from 2023-09-20 09-33-01.png">

## Understanding nodes:
• The ros2 node list will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them. 

<img src="images/Screenshot from 2023-09-20 10-20-32.png">

• ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node. 

<img src="images/Screenshot from 2023-09-20 10-21-02.png">

## Understanding topics:
•  ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

• The turtlesim tutorial tells you how to install rqt and all its plugins, including rqt_graph.

To run rqt_graph, open a new terminal and enter the command:

rqt_graph

<img src="images/Screenshot from 2023-09-23 19-27-28.png">

• Running the ros2 topic list command in a new terminal will return a list of all the topics currently active in the system:
• ros2 topic list -t will return the same list of topics, this time with the topic type appended in brackets:
• Since we know that /teleop_turtle publishes data to /turtlesim over the /turtle1/cmd_vel topic, let’s use echo to introspect that topic:

<img src="images/Screenshot from 2023-09-23 19-41-28.png">

• Now return to rqt_graph and uncheck the Debug box.

<img src="images/Screenshot from 2023-09-23 19-42-10.png">

• It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
--once is an optional argument meaning “publish one message then exit”.
And you will see your turtle move like so:

<img src="images/Screenshot from 2023-09-23 19-47-41.png">

• For one last introspection on this process, you can view the rate at which data is published using:

ros2 topic hz /turtle1/pose

<img src="images/Screenshot from 2023-09-23 19-52-09.png">

• The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. So, to get the turtle to keep moving, you can run:

ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

<img src="images/Screenshot from 2023-09-23 19-52-50.png">

## Understanding services:

• Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.


• To find out the type of a service, use the command:

ros2 service type <service_name>
• Let’s take a look at turtlesim’s /clear service. In a new terminal, enter the command:

ros2 service type /clear


<img src="images/Screenshot from 2023-09-24 21-24-49.png">


<img src="images/Screenshot from 2023-09-24 21-28-44.png">


## Understanding parameters:
• A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters. For more background on parameters, please see the concept document.


<img src="images/Screenshot from 2023-09-24 21-30-28.png">
<img src="images/Screenshot from 2023-09-24 21-33-31.png">

## Understanding actions:
• Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

• Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.


<img src="images/Screenshot from 2023-09-24 21-42-37.png">
<img src="images/Screenshot from 2023-09-24 21-42-47.png">
<img src="images/Screenshot from 2023-09-24 21-42-52.png">
## Using rqt_console to view logs:
• rqt_console is a GUI tool used to introspect log messages in ROS 2. Typically, log messages show up in your terminal. With rqt_console, you can collect those messages over time, view them closely and in a more organized manner, filter them, save them and even reload the saved files to introspect at a different time.

• Nodes use logs to output messages concerning events and status in a variety of ways. Their content is usually informational, for the sake of the user.


<img src="images/Screenshot from 2023-09-24 21-45-31.png">


## Launching nodes:
• In most of the introductory tutorials, you have been opening new terminals for every new node you run. As you create more complex systems with more and more nodes running simultaneously, opening terminals and reentering configuration details becomes tedious.

• Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

• Running a single launch file with the ros2 launch command will start up your entire system - all nodes and their configurations - at once.

<img src="images/Screenshot from 2023-09-24 21-48-22.png">

## Recording and playing back data:
• ros2 bag is a command line tool for recording data published on topics in your system. It accumulates the data passed on any number of topics and saves it in a database. You can then replay the data to reproduce the results of your tests and experiments. Recording topics is also a great way to share your work and allow others to recreate it.


# Beginner: Client libraries:

## Using colcon to build packages:
• colcon is an iteration on the ROS build tools catkin_make, catkin_make_isolated, catkin_tools and ament_tools. For more information on the design of colcon see this document. https://design.ros2.org/articles/build_tool.html

The source code can be found in the colcon GitHub organization.

<img src="images/Screenshot from 2023-09-27 00-51-06.png">

## Creating a workspace:
• A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

• You also have the option of sourcing an “overlay” - a secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending, or “underlay”. Your underlay must contain the dependencies of all the packages in your overlay. Packages in your overlay will override packages in the underlay. It’s also possible to have several layers of underlays and overlays, with each successive overlay using the packages of its parent underlays.


<img src="images/Screenshot from 2023-09-27 00-51-29.png">
<img src="images/Screenshot from 2023-09-27 00-55-55.png">

## Creating a package:
• A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

• Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python, which are officially supported, though other build types do exist.

<img src="images/Screenshot from 2023-09-30 12-25-12.png">
<img src="images/Screenshot from 2023-09-30 12-30-56.png">

## Writing a simple publisher and subscriber (C++):
• Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

• The code used in these examples can be found here. https://github.com/ros2/examples/tree/humble/rclcpp/topics

<img src="images/Screenshot from 2023-09-30 12-37-56.png">
<img src="images/Screenshot from 2023-09-30 12-47-15.png">
<img src="images/Screenshot from 2023-09-30 12-47-20.png">

## Writing a simple publisher and subscriber (Python):
• In this tutorial, you will create nodes that pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

• The code used in these examples can be found here. https://github.com/ros2/examples/tree/humble/rclpy/topics


## Writing a simple service and client (C++):
• When nodes communicate using services, the node that sends a request for data is called the client node, and the one that responds to the request is the service node. The structure of the request and response is determined by a .srv file.

• The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

 

## Writing a simple service and client (Python):
• When nodes communicate using services, the node that sends a request for data is called the client node, and the one that responds to the request is the service node. The structure of the request and response is determined by a .srv file.


• The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

<img src="images/Screenshot from 2023-09-30 12-58-21.png">
<img src="images/Screenshot from 2023-09-30 12-58-24.png">
<img src="images/Screenshot from 2023-09-30 12-58-27.png">
<img src="images/Screenshot from 2023-09-30 13-00-28.png">
<img src="images/Screenshot from 2023-09-30 13-00-38.png">
<img src="images/Screenshot from 2023-09-30 13-00-56.png">
<img src="images/Screenshot from 2023-09-30 13-01-01.png">

## Creating custom msg and srv files:
• In previous tutorials you utilized message and service interfaces to learn about topics, services, and simple publisher/subscriber (C++/Python) and service/client (C++/Python) nodes. The interfaces you used were predefined in those cases.

•  While it’s good practice to use predefined interface definitions, you will probably need to define your own messages and services sometimes as well. This tutorial will introduce you to the simplest method of creating custom interface definitions.



## Implementing custom interfaces:
• While best practice is to declare interfaces in dedicated interface packages, sometimes it can be convenient to declare, create and use an interface all in one package.

•  Recall that interfaces can currently only be defined in CMake packages. It is possible, however, to have Python libraries and nodes in CMake packages (using ament_cmake_python), so you could define interfaces and Python nodes together in one package. We’ll use a CMake package and C++ nodes here for the sake of simplicity.



## Using parameters in a class (C++):
• When making your own nodes you will sometimes need to add parameters that can be set from the launch file.

• This tutorial will show you how to create those parameters in a C++ class, and how to set them in a launch file.

• 

## Using parameters in a class (Python):
• When making your own nodes you will sometimes need to add parameters that can be set from the launch file.

• This tutorial will show you how to create those parameters in a Python class, and how to set them in a launch file.

<img src="images/Screenshot from 2023-09-30 13-37-44.png">
<img src="images/Screenshot from 2023-09-30 13-37-50.png">


<img src="images/Screenshot from 2023-09-30 13-38-44.png">
<img src="images/Screenshot from 2023-09-30 13-38-56.png">

## Using ros2doctor to identify issues:
• When your ROS 2 setup is not running as expected, you can check its settings with the ros2doctor tool.

• ros2doctor checks all aspects of ROS 2, including platform, version, network, environment, running systems and more, and warns you about possible errors and reasons for issues.

• 

## Creating and using plugins (C++):
• This tutorial is derived from http://wiki.ros.org/pluginlib and Writing and Using a Simple Plugin Tutorial.

• pluginlib is a C++ library for loading and unloading plugins from within a ROS package. Plugins are dynamically loadable classes that are loaded from a runtime library (i.e. shared object, dynamically linked library). With pluginlib, you do not have to explicitly link your application against the library containing the classes – instead pluginlib can open a library containing exported classes at any point without the application having any prior awareness of the library or the header file containing the class definition. Plugins are useful for extending/modifying application behavior without needing the application source code.

<img src="images/Screenshot from 2023-09-30 14-09-20.png">
<img src="images/Screenshot from 2023-09-30 14-09-34.png">
<img src="images/Screenshot from 2023-09-30 14-09-39.png">
<img src="images/Screenshot from 2023-09-30 14-09-41.png">
<img src="images/Screenshot from 2023-09-30 14-09-45.png">
<img src="images/Screenshot from 2023-09-30 14-09-47.png">


Intermediate:
Managing Dependencies with rosdep:
Creating an action:
Writing an action server and client (C++):
Writing an action server and client (Python):
Composing multiple nodes in a single process:
Monitoring for parameter changes (C++):

Launch:   
Creating a launch file
Integrating launch files into ROS 2 packages
Using substitutions
Using event handlers

   
tf2:
Introducing tf2
Writing a static broadcaster (Python)
Writing a static broadcaster (C++)
Writing a broadcaster (Python)
Writing a broadcaster (C++)
Writing a listener (Python)
Writing a listener (C++)
Adding a frame (Python)
Adding a frame (C++)
Using time (Python)
Using time (C++)
Traveling in time (Python)
Traveling in time (C++)
Debugging
Quaternion fundamentals
Using stamped datatypes with tf2_ros::MessageFilter

Testing:
Running Tests in ROS 2 from the Command Line
Writing Basic Tests with C++ with GTest
Writing Basic Tests with Python


URDF:
Building a visual robot model from scratch
Building a movable robot model
Adding physical and collision properties
Using Xacro to clean up your code
Using URDF with robot_state_publisher


# ROS 2 Intermediate Tutorials

## Managing Dependencies with rosdep

## Creating an Action

### Writing an Action Server and Client (C++)
### Writing an Action Server and Client (Python)

## Composing Multiple Nodes in a Single Process

## Monitoring for Parameter Changes (C++)

# ROS 2 Launch Tutorials

## Creating a Launch File
## Integrating Launch Files into ROS 2 Packages
## Using Substitutions
## Using Event Handlers

# ROS 2 tf2 Tutorials

## Introducing tf2
## Writing a Static Broadcaster (Python)
## Writing a Static Broadcaster (C++)
## Writing a Broadcaster (Python)
## Writing a Broadcaster (C++)
## Writing a Listener (Python)
## Writing a Listener (C++)
## Adding a Frame (Python)
## Adding a Frame (C++)
## Using Time (Python)
## Using Time (C++)
## Traveling in Time (Python)
## Traveling in Time (C++)
## Debugging
## Quaternion Fundamentals
## Using Stamped Datatypes with tf2_ros::MessageFilter

# ROS 2 Testing Tutorials

## Running Tests in ROS 2 from the Command Line
## Writing Basic Tests with C++ using GTest
## Writing Basic Tests with Python

# ROS 2 URDF Tutorials

## Building a Visual Robot Model from Scratch
## Building a Movable Robot Model
## Adding Physical and Collision Properties
## Using Xacro to Clean Up Your Code
## Using URDF with robot_state_publisher

