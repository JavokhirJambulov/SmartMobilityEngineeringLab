# SmartMobilityEngineeringLab
ROS2 Tutorials

i. [Beginner: CLI Tools](beginner-cli-tools)
   1. [Configuring environment](#configuring-environment)
   2. [Using turtlesim, ros2, and rqt](#using-turtlesim-ros2-and-rqt)
   3. [Understanding nodes](#understanding-nodes)
   4. [Understanding topics](#understanding-topics)
   5. [Understanding services](#understanding-services)
   6. [Understanding parameters](#understanding-parameters)
   7. [Understanding actions](#understanding-actions)
   8. [Using rqt_console to view logs]()
   9. [Launching nodes](#launching-nodes)
   10. [Recording and playing back data](#recording-and-playing-back-data)

ii. [Beginner: Client libraries](#beginner-client-libraries)
   1. [Using Colcon to Build Packages](#using-colcon-to-build-packages)
   2. [Creating a Workspace](#creating-a-workspace)
   3. [Creating a Package](#creating-a-package)
   4. [Writing a Simple Publisher and Subscriber (C++)](##writing-a-simple-publisher-and-subscriber-c)
   5. [Writing a Simple Publisher and Subscriber (Python)](#writing-a-simple-publisher-and-subscriber-python)
   6. [Writing a Simple Service and Client (C++)](#writing-a-simple-service-and-client-c)
   7. [Writing a Simple Service and Client (Python)](#writing-a-simple-service-and-client-python)
   8. [Creating Custom Msg and Srv Files](#creating-custom-msg-and-srv-files)
   9. [Implementing Custom Interfaces](#implementing-custom-interfaces)
   10. [Using Parameters in a Class (C++)](#using-parameters-in-a-class-c)
   11. [Using Parameters in a Class (Python)](#using-parameters-in-a-class-python)
   12. [Using ros2doctor to Identify Issues](#using-ros2doctor-to-identify-issues)
   13. [Creating and Using Plugins (C++)](#creating-and-using-plugins-c)

iii. [# ROS 2 Intermediate Tutorials](#ros-2-intermediate-tutorials)
   1. [Managing Dependencies with rosdep](#managing-dependencies-with-rosdep)
   2. [Creating an Action](#creating-an-action)
   3. [Writing an Action Server and Client (C++)](#writing-an-action-server-and-client-c)
   4. [Writing an Action Server and Client (Python)](#writing-an-action-server-and-client-python)
   5. [Composing Multiple Nodes in a Single Process](#composing-multiple-nodes-in-a-single-process)
   6. [Monitoring for Parameter Changes (C++)](#monitoring-for-parameter-changes-c)
   7. [ROS 2 Launch Tutorials](#ros-2-launch-tutorials)
   8. [Creating a Launch File](#creating-a-launch-file)
   9. [Integrating Launch Files into ROS 2 Packages](#integrating-launch-files-into-ros-2-packages)
   10. [Using Substitutions](#using-substitutions)
   11. [Using Event Handlers](#using-event-handlers)
   12. [ROS 2 tf2 Tutorials](#ros-2-tf2-tutorials)
   13. [Introducing tf2](#introducing-tf2)
   14. [Writing a Static Broadcaster (Python)](#writing-a-static-broadcaster-python)
   15. [Writing a Static Broadcaster (C++)](#writing-a-static-broadcaster-c)
   16. [Writing a Broadcaster (Python)](#writing-a-broadcaster-python)
   17. [Writing a Broadcaster (C++)](#writing-a-broadcaster-c)
   18. [Writing a Listener (Python)](#writing-a-listener-python)
   19. [Writing a Listener (C++)](#writing-a-listener-c)
   20. [Adding a Frame (Python)](#adding-a-frame-python)
   21. [Adding a Frame (C++)](#adding-a-frame-c)
   22. [Using Time (Python)](#using-time-python)
   23. [Using Time (C++)](#using-time-c)
   24. [Traveling in Time (Python)](#traveling-in-time-python)
   25. [Traveling in Time (C++)](#traveling-in-time-c)
   26. [Debugging](#debugging)
   27. [Quaternion Fundamentals](#quaternion-fundamentals)
   28. [Using Stamped Datatypes with tf2_ros::MessageFilter](#using-stamped-datatypes-with-tf2_rosmessagefilter)
   29. [ROS 2 Testing Tutorials](#ros-2-testing-tutorials)
   30. [Running Tests in ROS 2 from the Command Line](#running-tests-in-ros-2-from-the-command-line)
   31. [Writing Basic Tests with C++ using GTest](#writing-basic-tests-with-c-using-gtest)
   32. [Writing Basic Tests with Python](#writing-basic-tests-with-python)
   33. [ROS 2 URDF Tutorials](#ros-2-urdf-tutorials)
   34. [Building a Visual Robot Model from Scratch](#building-a-visual-robot-model-from-scratch)
   35. [Building a Movable Robot Model](#building-a-movable-robot-model)
   36. [Adding Physical and Collision Properties](#adding-physical-and-collision-properties)
   37. [Using Xacro to Clean Up Your Code](#using-xacro-to-clean-up-your-code)
   38. [Using URDF with robot_state_publisher](#using-urdf-with-robot_state_publisher)




# Beginner: CLI Tools:
• This tutorial will show you how to prepare your ROS 2 environment

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


# ROS 2 Intermediate Tutorials

## Managing Dependencies with rosdep
- rosdep is a dependency management utility that can work with packages and external libraries. It is a command-line utility for identifying and installing dependencies to build or install a package. rosdep is not a package manager in its own right; it is a meta-package manager that uses its own knowledge of the system and the dependencies to find the appropriate package to install on a particular platform. The actual installation is done using the system package manager (e.g. apt on Debian/Ubuntu, dnf on Fedora/RHEL, etc).
- The package.xml is the file in your software where rosdep finds the set of dependencies. It is important that the list of dependencies in the package.xml is complete and correct, which allows all of the tooling to determine the packages dependencies. Missing or incorrect dependencies can lead to users not being able to use your package, to packages in a workspace being built out-of-order, and to packages not being able to be released.
- If your library isn’t in rosdistro, you can experience the greatness that is open-source software development: you can add it yourself! Pull requests for rosdistro are typically merged well within a week.


## Creating an Action

- You learned about actions previously in the Understanding actions tutorial. Like the other communication types and their respective interfaces (topics/msg and services/srv), you can also custom-define actions in your packages. This tutorial shows you how to define and build an action that you can use with the action server and action client you will write in the next tutorial.
- Prerequisites: You should have ROS 2 and colcon installed.

<img src="images/Screenshot from 2023-10-01 12-22-31.png">
<img src="images/Screenshot from 2023-10-01 12-22-40.png">
<img src="images/Screenshot from 2023-10-01 12-22-49.png">

## Writing an Action Server and Client (C++)

- Actions are a form of asynchronous communication in ROS. Action clients send goal requests to action servers. Action servers send goal feedback and results to action clients.
-Prerequisites:You will need the action_tutorials_interfaces package and the Fibonacci.action interface defined in the previous tutorial, Creating an action.

<img src="images/Screenshot from 2023-10-01 12-26-57.png">
<img src="images/Screenshot from 2023-10-01 12-28-11.png">
<img src="images/Screenshot from 2023-10-01 12-29-19.png">
<img src="images/Screenshot from 2023-10-01 12-32-17.png">
<img src="images/Screenshot from 2023-10-01 12-33-24.png">
<img src="images/Screenshot from 2023-10-01 12-34-55.png">
<img src="images/Screenshot from 2023-10-01 12-35-04.png">

## Writing an Action Server and Client (Python)

- Actions are a form of asynchronous communication in ROS 2. Action clients send goal requests to action servers. Action servers send goal feedback and results to action clients.

- Prerequisites: You will need the action_tutorials_interfaces package and the Fibonacci.action interface defined in the previous tutorial, Creating an action.

## Composing Multiple Nodes in a Single Process

- Discover available components
- To see what components are registered and available in the workspace, execute the following in a shell:
ros2 component types

-Run-time composition using ROS services with a publisher and subscriber
- In the first shell, start the component container:
ros2 run rclcpp_components component_container

- Open the second shell and verify that the container is running via ros2 command line tools:
ros2 component list

<img src="images/Screenshot from 2023-10-01 15-09-45.png">
<img src="images/Screenshot from 2023-10-01 18-00-21.png">
<img src="images/Screenshot from 2023-10-01 18-00-25.png">
<img src="images/Screenshot from 2023-10-01 18-01-00.png">
<img src="images/Screenshot from 2023-10-01 18-01-12.png">
<img src="images/Screenshot from 2023-10-01 18-01-35.png">

## Monitoring for Parameter Changes (C++)

- Often a node needs to respond to changes to its own parameters or another node’s parameters. The ParameterEventHandler class makes it easy to listen for parameter changes so that your code can respond to them. This tutorial will show you how to use the C++ version of the ParameterEventHandler class to monitor for changes to a node’s own parameters as well as changes to another node’s parameters.

- Before starting this tutorial, you should first complete the following tutorials:
Understanding parameters
Using parameters in a class (C++)
- In addition, you must be running the Galactic distribution of ROS 2.
- 
<img src="images/Screenshot from 2023-10-01 18-07-01.png">
<img src="images/Screenshot from 2023-10-01 18-07-13.png">
<img src="images/Screenshot from 2023-10-01 18-07-19.png">

# ROS 2 Launch Tutorials

- ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.
  
## Creating a Launch File

- The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.
  

<img src="images/Screenshot from 2023-10-01 18-09-35.png">
<img src="images/Screenshot from 2023-10-01 18-10-19.png">
<img src="images/Screenshot from 2023-10-01 18-10-53.png">
<img src="images/Annotation 2023-10-01 181118.png">

## Integrating Launch Files into ROS 2 Packages

- Prerequisites: You should have gone through the tutorial on how to create a ROS 2 package. As always, don’t forget to source ROS 2 in every new terminal you open.

- Background: In the previous tutorial, we saw how to write a standalone launch file. This tutorial will show how to add a launch file to an existing package, and the conventions typically used.

<img src="images/Screenshot from 2023-10-01 18-18-11.png">
<img src="images/Screenshot from 2023-10-01 18-18-16.png">

## Using Substitutions

- Launch files are used to start nodes, services, and execute processes. This set of actions may have arguments, which affect their behavior. Substitutions can be used in arguments to provide more flexibility when describing reusable launch files. Substitutions are variables that are only evaluated during the execution of the launch description and can be used to acquire specific information like a launch configuration, an environment variable, or to evaluate an arbitrary Python expression.

- This tutorial shows usage examples of substitutions in ROS 2 launch files.

## Using Event Handlers

- Launch in ROS 2 is a system that executes and manages user-defined processes. It is responsible for monitoring the state of processes it launched, as well as reporting and reacting to changes in the state of those processes. These changes are called events and can be handled by registering an event handler with the launch system. Event handlers can be registered for specific events and can be useful for monitoring the state of processes. Additionally, they can be used to define a complex set of rules which can be used to dynamically modify the launch file.

- This tutorial shows usage examples of event handlers in ROS 2 launch files.

## Managing Large projects

- This tutorial describes some tips for writing launch files for large projects. The focus is on how to structure launch files so they may be reused as much as possible in different situations. Additionally, it covers usage examples of different ROS 2 launch tools, like parameters, YAML files, remappings, namespaces, default arguments, and RViz configs.

<img src="images/Annotation 2023-10-01 182010.png">

<img src="images/Annotation 2023-10-01 182024.png">

# ROS 2 tf2 Tutorials

- Many of the tf2 tutorials are available for both C++ and Python. The tutorials are streamlined to complete either the C++ track or the Python track. If you want to learn both C++ and Python, you should go through the tutorials once for C++ and once for Python.

## Introducing tf2

- Let’s start by installing the demo package and its dependencies.
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations

- Now that we’ve installed the turtle_tf2_py tutorial package let’s run the demo. First, open a new terminal and source your ROS 2 installation so that ros2 commands will work. Then run the following command:
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py

<img src="images/Screenshot from 2023-10-01 18-23-44.png">
<img src="images/Screenshot from 2023-10-01 18-24-30.png">
<img src="images/Screenshot from 2023-10-01 18-26-12.png">
<img src="images/Screenshot from 2023-10-01 18-26-24.png">
<img src="images/Screenshot from 2023-10-01 18-28-14.png">

## Writing a Static Broadcaster (Python)

- Publishing static transforms is useful to define the relationship between a robot base and its sensors or non-moving parts. For example, it is easiest to reason about laser scan measurements in a frame at the center of the laser scanner.

- This is a standalone tutorial covering the basics of static transforms, which consists of two parts. In the first part we will write code to publish static transforms to tf2. In the second part, we will explain how to use the commandline static_transform_publisher executable tool in tf2_ros.

<img src="images/Screenshot from 2023-10-01 18-35-59.png">

## Writing a Static Broadcaster (C++)

- Publishing static transforms is useful to define the relationship between a robot base and its sensors or non-moving parts. For example, it is easiest to reason about laser scan measurements in a frame at the center of the laser scanner.

- This is a standalone tutorial covering the basics of static transforms, which consists of two parts. In the first part we will write code to publish static transforms to tf2. In the second part, we will explain how to use the commandline static_transform_publisher executable tool in tf2_ros.
  
## Writing a Broadcaster (Python)

- In the next two tutorials we will write the code to reproduce the demo from the Introduction to tf2 tutorial. After that, following tutorials focus on extending the demo with more advanced tf2 features, including the usage of timeouts in transformation lookups and time travel.

## Writing a Broadcaster (C++)

- In the next two tutorials we will write the code to reproduce the demo from the Introduction to tf2 tutorial. After that, following tutorials focus on extending the demo with more advanced tf2 features, including the usage of timeouts in transformation lookups and time travel.

## Writing a Listener (Python)

- In previous tutorials we created a tf2 broadcaster to publish the pose of a turtle to tf2.

- In this tutorial, we’ll create a tf2 listener to start using tf2.
  
- This tutorial assumes you have completed the tf2 broadcaster tutorial (Python). In the previous tutorial, we created a learning_tf2_py package, which is where we will continue working from.

## Writing a Listener (C++)
- In previous tutorials we created a tf2 broadcaster to publish the pose of a turtle to tf2.

- In this tutorial, we’ll create a tf2 listener to start using tf2.
  
- This tutorial assumes you have completed the tf2 static broadcaster tutorial (C++) and the tf2 broadcaster tutorial (C++). In the previous tutorial, we created a learning_tf2_cpp package, which is where we will continue working from.
  
## Adding a Frame (Python)

- In previous tutorials, we recreated the turtle demo by writing a tf2 broadcaster and a tf2 listener. This tutorial will teach you how to add extra fixed and dynamic frames to the transformation tree. In fact, adding a frame in tf2 is very similar to creating the tf2 broadcaster, but this example will show you some additional features of tf2.

- For many tasks related to transformations, it is easier to think inside a local frame. For example, it is easiest to reason about laser scan measurements in a frame at the center of the laser scanner. tf2 allows you to define a local frame for each sensor, link, or joint in your system. When transforming from one frame to another, tf2 will take care of all the hidden intermediate frame transformations that are introduced.

## Adding a Frame (C++)

- In previous tutorials, we recreated the turtle demo by writing a tf2 broadcaster and a tf2 listener. This tutorial will teach you how to add extra fixed and dynamic frames to the transformation tree. In fact, adding a frame in tf2 is very similar to creating the tf2 broadcaster, but this example will show you some additional features of tf2.

- For many tasks related to transformations, it is easier to think inside a local frame. For example, it is easiest to reason about laser scan measurements in a frame at the center of the laser scanner. tf2 allows you to define a local frame for each sensor, link, or joint in your system. When transforming from one frame to another, tf2 will take care of all the hidden intermediate frame transformations that are introduced.

## Using Time (Python)

- In previous tutorials, we recreated the turtle demo by writing a tf2 broadcaster and a tf2 listener. We also learned how to add a new frame to the transformation tree. Now we will learn more about the timeout argument which makes the lookup_transform wait for the specified transform for up to the specified duration before throwing an exception. This tool can be useful to listen for transforms that are published at varying rates or those incoming source with unreliable networking and non negligible latency. This tutorial will teach you how use the timeout in lookup_transform function to wait for a transform to be available on the tf2 tree.

- You should notice that lookup_transform() will actually block until the transform between the two turtles becomes available (this will usually take a few milli-seconds). Once the timeout has been reached (one second in this case), an exception will be raised only if the transform is still not available.

- In this tutorial you learned more about the lookup_transform function and its timeout features. You also learned how to catch and handle additional exceptions that can be thrown by tf2.


## Using Time (C++)

- In previous tutorials, we recreated the turtle demo by writing a tf2 broadcaster and a tf2 listener. We also learned how to add a new frame to the transformation tree. Now we will learn more about the timeout argument which makes the lookup_transform wait for the specified transform for up to the specified duration before throwing an exception. This tool can be useful to listen for transforms that are published at varying rates or those incoming source with unreliable networking and non negligible latency. This tutorial will teach you how use the timeout in lookup_transform function to wait for a transform to be available on the tf2 tree.

- You should notice that lookup_transform() will actually block until the transform between the two turtles becomes available (this will usually take a few milli-seconds). Once the timeout has been reached (one second in this case), an exception will be raised only if the transform is still not available.

- In this tutorial you learned more about the lookup_transform function and its timeout features. You also learned how to catch and handle additional exceptions that can be thrown by tf2.

## Traveling in Time (Python)

- In the previous tutorial, we discussed the basics of tf2 and time. This tutorial will take us one step further and expose a powerful tf2 trick: the time travel. In short, one of the key features of tf2 library is that it is able to transform data in time as well as in space.

- This tf2 time travel feature can be useful for various tasks, like monitoring the pose of the robot for a long period of time or building a follower robot that will follow the “steps” of the leader. We will use that time travel feature to look up transforms back in time and program turtle2 to follow 5 seconds behind carrot1.

- In this tutorial, you have seen one of the advanced features of tf2. You learned that tf2 can transform data in time and learned how to do that with turtlesim example. tf2 allowed you to go back in time and make frame transformations between old and current poses of turtles by using the advanced lookup_transform_full() API.

## Traveling in Time (C++)

- In the previous tutorial, we discussed the basics of tf2 and time. This tutorial will take us one step further and expose a powerful tf2 trick: the time travel. In short, one of the key features of tf2 library is that it is able to transform data in time as well as in space.

- This tf2 time travel feature can be useful for various tasks, like monitoring the pose of the robot for a long period of time or building a follower robot that will follow the “steps” of the leader. We will use that time travel feature to look up transforms back in time and program turtle2 to follow 5 seconds behind carrot1.

- In this tutorial, you have seen one of the advanced features of tf2. You learned that tf2 can transform data in time and learned how to do that with turtlesim example. tf2 allowed you to go back in time and make frame transformations between old and current poses of turtles by using the advanced lookup_transform_full() API.

## Debugging

- This tutorial walks you through the steps to debug a typical tf2 problem. It will also use many of the tf2 debugging tools, such as tf2_echo, tf2_monitor, and view_frames. This tutorial assumes you have completed the learning tf2 tutorials.

- In this tutorial, you learned how to use a systematic approach for debugging tf2 related problems. You also learned how to use tf2 debugging tools, such as tf2_echo, tf2_monitor, and view_frames to help you debug those tf2 problems.


## Quaternion Fundamentals

- A quaternion is a 4-tuple representation of orientation, which is more concise than a rotation matrix. Quaternions are very efficient for analyzing situations where rotations in three dimensions are involved. Quaternions are used widely in robotics, quantum mechanics, computer vision, and 3D animation.         

- In this tutorial, you will learn how quaternions and conversion methods work in ROS 2. However, this is not a hard requirement and you can stick to any other geometric transfromation library that suit you best.

- In this tutorial, you learned about the fundamental concepts of a quaternion and its related mathematical operations, like inversion and rotation. You also learned about its usage examples in ROS 2 and conversion methods between two separate Quaternion classes.


## Using Stamped Datatypes with tf2_ros::MessageFilter

- This tutorial explains how to use sensor data with tf2. Some real-world examples of sensor data are:
cameras, both mono and stereo
laser scans
- Suppose that a new turtle named turtle3 is created and it doesn’t have good odometry, but there is an overhead camera tracking its position and publishing it as a PointStamped message in relation to the world frame.

- turtle1 wants to know where turtle3 is compared to itself.

- To do this turtle1 must listen to the topic where turtle3’s pose is being published, wait until transforms into the desired frame are ready, and then do its operations. To make this easier the tf2_ros::MessageFilter is very useful. The tf2_ros::MessageFilter will take a subscription to any ROS 2 message with a header and cache it until it is possible to transform it into the target frame.

- In this tutorial you learned how to use sensor data/messages in tf2. Specifically speaking, you learned how to publish PointStamped messages on a topic, and how to listen to the topic and transform the frame of PointStamped messages with tf2_ros::MessageFilter.


# ROS 2 Testing Tutorials


## Running Tests in ROS 2 from the Command Line

- Build and run your tests
- To compile and run the tests, simply run the test verb from colcon.
- colcon test --ctest-args tests [package_selection_args]
- (where package_selection_args are optional package selection arguments for colcon to limit which packages are built and run)
- Sourcing the workspace before testing should not be necessary. colcon test makes sure that the tests run with the right environment, have access to their dependencies, etc.

<img src="images/Screenshot from 2023-10-01 18-38-32.png">

## Writing Basic Tests with C++ using GTest

- Starting point: we’ll assume you have a basic ament_cmake package set up already and you want to add some tests to it.

- In this tutorial, we’ll be using gtest.

## Writing Basic Tests with Python

- Starting point: we’ll assume you have a basic ament_python package set up already and you want to add some tests to it.

- If you are using ament_cmake_python, refer to the the ament_cmake_python docs for how to make tests dicoverable. The test contents and invocation with colcon remain the same.

# ROS 2 URDF Tutorials

- URDF (Unified Robot Description Format) is a file format for specifying the geometry and organization of robots in ROS.

## Building a Visual Robot Model from Scratch

- In this tutorial, we’re going to build a visual model of a robot that vaguely looks like R2D2. In later tutorials, you’ll learn how to articulate the model, add in some physical properties, and generate neater code with xacro, but for now, we’re going to focus on getting the visual geometry correct.

- Before continuing, make sure you have the joint_state_publisher package installed. If you installed urdf_tutorial binaries, this should already be the case. If not, please update your installation to include that package (use rosdep to check).

<img src="images/Annotation 2023-10-01 184148.png">
<img src="images/Annotation 2023-10-01 184201.png">
<img src="images/Annotation 2023-10-01 184212.png">
<img src="images/Annotation 2023-10-01 184240.png">

## Building a Movable Robot Model

- In this tutorial, we’re going to revise the R2D2 model we made in the previous tutorial so that it has movable joints. In the previous model, all of the joints were fixed. Now we’ll explore three other important types of joints: continuous, revolute and prismatic.

- Make sure you have installed all prerequisites before continuing. See the previous tutorial for information on what is required.

## Adding Physical and Collision Properties

- In this tutorial, we’ll look at how to add some basic physical properties to your URDF model and how to specify its collision properties.
- So far, we’ve only specified our links with a single sub-element, visual, which defines (not surprisingly) what the robot looks like. However, in order to get collision detection to work or to simulate the robot, we need to define a collision element as well. Here is the new urdf with collision and physical properties.

## Using Xacro to Clean Up Your Code

- By now, if you’re following all these steps at home with your own robot design, you might be sick of doing all sorts of math to get very simple robot descriptions to parse correctly. Fortunately, you can use the xacro package to make your life simpler. It does three things that are very helpful.
Constants
Simple Math
Macros

- In this tutorial, we take a look at all these shortcuts to help reduce the overall size of the URDF file and make it easier to read and maintain.

## Using URDF with robot_state_publisher

- This tutorial will show you how to model a walking robot, publish the state as a tf2 message and view the simulation in Rviz. First, we create the URDF model describing the robot assembly. Next we write a node which simulates the motion and publishes the JointState and transforms. We then use robot_state_publisher to publish the entire robot state to /tf2.
- Prerequisites
rviz2

- As always, don’t forget to source ROS 2 in every new terminal you open.
- 
<img src="images/Annotation 2023-10-01 184309.png">







