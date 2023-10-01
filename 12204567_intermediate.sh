#12204567 Javokhir ROS2 Tutorials Intermediate

# -------ROS 2 Intermediate Tutorials ---------

# --------- Managing Dependencies with rosdep ---------

#rosdep installation
#If you are using rosdep with ROS, it is conveniently packaged along with the ROS distribution. This is the recommended way to get rosdep. You can install it with:

sudo apt-get install python3-rosdep

#Firstly, if this is the first time using rosdep, it must be initialized via:

sudo rosdep init
rosdep update


## --------- Creating an Action ---------

You should have ROS 2 and colcon installed.

Set up a workspace and create a package named action_tutorials_interfaces:

(Remember to source your ROS 2 installation first.)

mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces

#Create an action directory in our ROS 2 package action_tutorials_interfaces:
cd action_tutorials_interfaces
mkdir action

#Within the action directory, create a file called Fibonacci.action with the following contents:
int32 order
---
int32[] sequence
---
int32[] partial_sequence

#This is accomplished by adding the following lines to our CMakeLists.txt before the ament_package() line, in the action_tutorials_interfaces:
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

#We should also add the required dependencies to our package.xml:

<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>

#We should now be able to build the package containing the Fibonacci action definition:

# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build

#We can check that our action built successfully with the command line tool:

# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci

## --------- Writing an Action Server and Client (C++) ---------

#Goal: Implement an action server and client in C++.

#Creating the action_tutorials_cpp package
#Go into the action workspace you created in the previous tutorial (remember to source the workspace), and create a new package for the C++ action server:
cd ~/ros2_ws/src
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

#And now we can compile the package. Go to the top-level of the ros2_ws, and run:
colcon build

#Now that we have the action server built, we can run it. Source the workspace we just built (ros2_ws), and try to run the action server:
ros2 run action_tutorials_cpp fibonacci_action_server

#And now we can compile the package. Go to the top-level of the ros2_ws, and run:
colcon build

#Now that we have the action client built, we can run it. First make sure that an action server is running in a separate terminal. Now source the workspace we just built (ros2_ws), and try to run the action client:
ros2 run action_tutorials_cpp fibonacci_action_client

#You should see logged messages for the goal being accepted, feedback being printed, and the final result.


## --------- Writing an Action Server and Client (Python) ---------

#Open a new file in your home directory, let’s call it fibonacci_action_server.py, and add the following code:
```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```
#Let’s try running our action server:
python3 fibonacci_action_server.py
#In another terminal, we can use the command line interface to send a goal:
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

#Now let’s make our goal execution actually compute and return the requested Fibonacci sequence:
```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result
```

#Again, restart the action server and send another goal. You should see the goal finish with the proper result sequence.
python3 fibonacci_action_server.py
#In another terminal, we can use the command line interface to send a goal:
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

#We’ll also scope the action client to a single file. Open a new file, let’s call it fibonacci_action_client.py, and add the following boilerplate code:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

#Let’s test our action client by first running the action server built earlier:
python3 fibonacci_action_server.py
#In another terminal, run the action client:
python3 fibonacci_action_client.py
#You should see messages printed by the action server as it successfully executes the goal:
```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc
```
#With an action server running in a separate terminal, go ahead and try running our Fibonacci action client!
python3 fibonacci_action_client.py

#We’re all set. If we run our action client, you should see feedback being printed to the screen.



## --------- Composing Multiple Nodes in a Single Process ---------

#Discover available components
#To see what components are registered and available in the workspace, execute the following in a shell:
ros2 component types

#In the first shell, start the component container:
ros2 run rclcpp_components component_container
#Open the second shell and verify that the container is running via ros2 command line tools:
ros2 component list

#In the second shell load the talker component (see talker source code):
ros2 component load /ComponentManager composition composition::Talker

#The command will return the unique ID of the loaded component as well as the node name:
#Loaded component 1 into '/ComponentManager' container node as '/talker'

#Run another command in the second shell to load the listener component (see listener source code):
ros2 component load /ComponentManager composition composition::Listener
#Terminal will return:
Loaded component 2 into '/ComponentManager' container node as '/listener'
#The ros2 command line utility can now be used to inspect the state of the container:
ros2 component list
#You will see the following result:
/ComponentManager
   1  /talker
   2  /listener


#Run-time composition using ROS services with a server and client

#In the first shell:
ros2 run rclcpp_components component_container
#In the second shell (see server and client source code):
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client

#Compile-time composition using ROS services
#In the shell call (see source code):
ros2 run composition manual_composition
#This should show repeated messages from both pairs, the talker and the listener as well as the server and the client.

#Composition using launch actions
ros2 launch composition composition_demo.launch.py

#Unloading components
#In the first shell, start the component container:

ros2 run rclcpp_components component_container
#Verify that the container is running via ros2 command line tools:

ros2 component list
#You should see a name of the component:

/ComponentManager
#In the second shell load both the talker and listener as we have before:

ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
#Use the unique ID to unload the node from the component container.

ros2 component unload /ComponentManager 1 2
#The terminal should return:

Unloaded component 1 from '/ComponentManager' container
Unloaded component 2 from '/ComponentManager' container
#In the first shell, verify that the repeated messages from talker and listener have stopped.

#Passing parameter values into components
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true
#Passing additional arguments into components
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true


## --------- Monitoring for Parameter Changes (C++) ---------

#First, open a new terminal and source your ROS 2 installation so that ros2 commands will work.
. install/setup.bash
# navigate into ros2_ws/src and then create a new package there:
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp

#Update package.xmlto add the description, maintainer email and name, and license information to package.xml
<description>C++ parameter events client tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>

#Build and run
#It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
#Navigate back to the root of your workspace, ros2_ws, and build your new package:
colcon build --packages-select cpp_parameter_event_handler

#Open a new terminal, navigate to ros2_ws, and source the setup files:
. install/setup.bash
#Now run the node:
ros2 run cpp_parameter_event_handler parameter_event_handler

#In a terminal, navigate back to the root of your workspace, ros2_ws, and build your updated package as before:
colcon build --packages-select cpp_parameter_event_handler

#Then source the setup files:
. install/setup.bash

#Now, to test monitoring of remote parameters, first run the newly-built parameter_event_handler code:
ros2 run cpp_parameter_event_handler parameter_event_handler
#Next, from another teminal (with ROS initialized), run the parameter_blackboard demo application, as follows:
ros2 run demo_nodes_cpp parameter_blackboard
#Finally, from a third terminal (with ROS initialized), let’s set a parameter on the parameter_blackboard node:
ros2 param set parameter_blackboard a_double_param 3.45


# --------- ROS 2 Launch Tutorials ---------

 #ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

##  --------- Creating a Launch File --------- 

#1 Setup
#Create a new directory to store your launch files:
mkdir launch

#2 Write the launch file
#Let’s put together a ROS 2 launch file using the turtlesim package and its executables. As mentioned above, this can either be in Python, XML, or YAML.
#Copy and paste the complete code into the launch/turtlesim_mimic_launch.py file:
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
#To run the launch file created above, enter into the directory you created earlier and run the following command:
cd launch
ros2 launch turtlesim_mimic_launch.py

#Two turtlesim windows will open, and you will see the following [INFO] messages telling you which nodes your launch file has started:
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]

#To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

#4 Introspect the system with rqt_graph
#While the system is still running, open a new terminal and run rqt_graph to get a better idea of the relationship between the nodes in your launch file.
rqt_graph

#A hidden node (the ros2 topic pub command you ran) is publishing data to the /turtlesim1/turtle1/cmd_vel topic on the left, which the /turtlesim1/sim node is subscribed to. 



##  --------- Integrating Launch Files into ROS 2 Packages --------- 

#Create a workspace for the package to live in:
mkdir -p launch_ws/src
cd launch_ws/src
ros2 pkg create py_launch_example --build-type ament_python

#2 Creating the structure to hold launch files
#In order for colcon to find the launch files, we need to inform Python’s setup tools of our launch files using the data_files parameter of setup.
#Inside our setup.py file:
```
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ]
)
```
#3 Writing the launch file
#Your launch file should define the generate_launch_description() function which returns a launch.LaunchDescription() to be used by the ros2 launch verb

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])

#4 Building and running the launch file
colcon build
#After the colcon build has been successful and you’ve sourced the workspace, you should be able to run the launch file as follows:
ros2 launch py_launch_example my_script_launch.py


##  --------- Using Substitutions ---------

#1 Create and setup the package
#Create a new package of build_type ament_python called launch_tutorial:
ros2 pkg create launch_tutorial --build-type ament_python

#Inside of that package, create a directory called launch:
mkdir launch_tutorial/launch

#Finally, make sure to add in changes to the setup.py of the package so that the launch files will be installed:
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ]
)

#4 Build the package
#Go to the root of the workspace, and build the package:
colcon build

#Also remember to source the workspace after building.

#Launching example
#Now you can launch the example_main.launch.py file using the ros2 launch command.
ros2 launch launch_tutorial example_main.launch.py

#Modifying launch arguments
ros2 launch launch_tutorial example_substitutions.launch.py --show-args

#Now you can pass the desired arguments to the launch file as follows:
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200


##  --------- Using Event Handlers ---------

#This tutorial uses the turtlesim package. This tutorial also assumes you have created a new package of build type ament_python called launch_tutorial.
#Create a new file called example_event_handlers.launch.py file in the launch folder of the launch_tutorial package.
#Go to the root of the workspace, and build the package:
colcon build

#Now you can launch the example_event_handlers.launch.py file using the ros2 launch command.
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

#This will do the following:
#Start a turtlesim node with a blue background
#Spawn the second turtle
#Change the color to purple
#Change the color to pink after two seconds if the provided background_r argument is 200 and use_provided_red argument is True
#Shutdown the launch file when the turtlesim window is closed



## ---------- Managin large projects - ---- -- -- 

#To finally see the result of our code, build the package and launch the top-level launch file using the following command:
ros2 launch launch_tutorial launch_turtlesim.launch.py

#You will now see the two turtlesim simulations started. There are two turtles in the first one and one in the second one. In the first simulation,
# turtle2 is spawned in the bottom-left part of the world. Its aim is to reach the carrot1 frame which is five meters away on the x-axis relative to the turtle1 frame.

#The turtlesim2/turtle1 in the second is designed to mimic the behavior of the turtle2.

#If you want to control the turtle1, run the teleop node.

ros2 run turtlesim turtle_teleop_key

#  --------- ROS 2 tf2 Tutorials --------- 

#Many of the tf2 tutorials are available for both C++ and Python. The tutorials are streamlined to complete either the
# C++ track or the Python track. If you want to learn both C++ and Python, you should go through the tutorials once for C++ and once for Python.



## ---------  Introducing tf2 --------- 

#Installing the demo
#Let’s start by installing the demo package and its dependencies.
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations

#Running the demo
#First, open a new terminal and source your ROS 2 installation so that ros2 commands will work. Then run the following command:
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py

#In the second terminal window type the following command:
ros2 run turtlesim turtle_teleop_key

#Now let’s look at how tf2 is being used to create this demo. We can use tf2_tools to look at what tf2 is doing behind the scenes.
#view_frames creates a diagram of the frames being broadcast by tf2 over ROS.
ros2 run tf2_tools view_frames

#You will see:

Listening to tf data during 5 seconds...
Generating graph in frames.pdf file...

#tf2_echo reports the transform between any two frames broadcasted over ROS.

#Usage:
ros2 run tf2_ros tf2_echo [source_frame] [target_frame]

#Let’s look at the transform of the turtle2 frame with respect to turtle1 frame which is equivalent to:
ros2 run tf2_ros tf2_echo turtle2 turtle1

#rviz and tf2
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz


## ---------  Writing a Static Broadcaster (Python) --------- 

#First we will create a package that will be used for this tutorial and the following ones. The package called learning_tf2_py will depend on geometry_msgs, python3-numpy, rclpy, tf2_ros_py, and turtlesim. Code for this tutorial is stored here.
#Open a new terminal and source your ROS 2 installation so that ros2 commands will work. Navigate to workspace’s src folder and create a new package:
ros2 pkg create --build-type ament_python learning_tf2_py

#Let’s first create the source files. Inside the src/learning_tf2_py/learning_tf2_py directory download the example static broadcaster code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py

#Add dependencies
#Open package.xml with your text editor.

#As mentioned in the Create a package tutorial, make sure to fill in the <description>, <maintainer> and <license> tags:

<description>Learning tf2 with rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>

#After the lines above, add the following dependencies corresponding to your node’s import statements:

<exec_depend>geometry_msgs</exec_depend>
<exec_depend>python3-numpy</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>tf2_ros_py</exec_depend>
<exec_depend>turtlesim</exec_depend>
#Make sure to save the file.

#Add an entry point
#Add the following line between the 'console_scripts': brackets:
'static_turtle_tf2_broadcaster = learning_tf2_py.static_turtle_tf2_broadcaster:main',

#It’s good practice to run rosdep in the root of your workspace to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro humble -y

#Still in the root of your workspace, build your new package:
colcon build --packages-select learning_tf2_py

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now run the static_turtle_tf2_broadcaster node:
ros2 run learning_tf2_py static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0

#We can now check that the static transform has been published by echoing the tf_static topic
ros2 topic echo /tf_static

#Publish a static coordinate transform to tf2 using an x/y/z offset in meters and roll/pitch/yaw in radians. In our case, roll/pitch/yaw refers to rotation about the x/y/z-axis, respectively.
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id

#Publish a static coordinate transform to tf2 using an x/y/z offset in meters and quaternion.
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id


##  --------- Writing a Static Broadcaster (C++) --------- 

#Open a new terminal and source your ROS 2 installation so that ros2 commands will work. Navigate to workspace’s src folder and create a new package:         
ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp

#Let’s first create the source files. Inside the src/learning_tf2_cpp/src directory download the example static broadcaster code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp

#Open package.xml with your text editor.
#As mentioned in the Create a package tutorial, make sure to fill in the <description>, <maintainer> and <license> tags:
<description>Learning tf2 with rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>

#It’s good practice to run rosdep in the root of your workspace to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro humble -y

#Still in the root of your workspace, build your new package:
colcon build --packages-select learning_tf2_cpp

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now run the static_turtle_tf2_broadcaster node:
ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0

#We can now check that the static transform has been published by echoing the tf_static topic
ros2 topic echo /tf_static

#Publish a static coordinate transform to tf2 using an x/y/z offset in meters and roll/pitch/yaw in radians. In our case, roll/pitch/yaw refers to rotation about the x/y/z-axis, respectively.
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id

#Publish a static coordinate transform to tf2 using an x/y/z offset in meters and quaternion.
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id

##  --------- Writing a Broadcaster (Python) --------- 

#Let’s first create the source files. Go to the learning_tf2_py package we created in the previous tutorial. Inside the src/learning_tf2_py/learning_tf2_py directory download the example broadcaster code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py

#To allow the ros2 run command to run your node, you must add the entry point to setup.py (located in the src/learning_tf2_py directory).
#Finally, add the following line between the 'console_scripts': brackets:
'turtle_tf2_broadcaster = learning_tf2_py.turtle_tf2_broadcaster:main',

#It’s good practice to run rosdep in the root of your workspace to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro humble -y

#Still in the root of your workspace, build your new package:
colcon build --packages-select learning_tf2_py

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now run the launch file that will start the turtlesim simulation node and turtle_tf2_broadcaster node:
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

#In the second terminal window type the following command:
ros2 run turtlesim turtle_teleop_key

#Now, use the tf2_echo tool to check if the turtle pose is actually getting broadcast to tf2:

ros2 run tf2_ros tf2_echo world turtle1

##  --------- Writing a Broadcaster (C++) --------- 

#Let’s first create the source files. Go to the learning_tf2_cpp package we created in the previous tutorial. Inside the src directory download the example broadcaster code by entering the following command:

wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_broadcaster.cpp

#Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#From the root of your workspace, build your updated package:
colcon build --packages-select learning_tf2_cpp

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now run the launch file that will start the turtlesim simulation node and turtle_tf2_broadcaster node:
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

#In the second terminal window type the following command:
ros2 run turtlesim turtle_teleop_key

#Now, use the tf2_echo tool to check if the turtle pose is actually getting broadcast to tf2:
ros2 run tf2_ros tf2_echo world turtle1

##  --------- Writing a Listener (Python) --------- 

#Let’s first create the source files. Go to the learning_tf2_py package we created in the previous tutorial. Inside the src/learning_tf2_py/learning_tf2_py directory download the example listener code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_listener.py

#To allow the ros2 run command to run your node, you must add the entry point to setup.py (located in the src/learning_tf2_py directory).

'turtle_tf2_listener = learning_tf2_py.turtle_tf2_listener:main',

##Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#From the root of your workspace, build your updated package:
colcon build --packages-select learning_tf2_cpp

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now you’re ready to start your full turtle demo:
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

#You should see the turtle sim with two turtles. In the second terminal window type the following command:
ros2 run turtlesim turtle_teleop_key

##  --------- Writing a Listener (C++) --------- 
                          
#Let’s first create the source files. Go to the learning_tf2_cpp package we created in the previous tutorial. Inside the src directory download the example listener code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp

#Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#From the root of your workspace, build your updated package:
colcon build --packages-select learning_tf2_cpp

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now you’re ready to start your full turtle demo:
ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py

#You should see the turtle sim with two turtles. In the second terminal window type the following command:
ros2 run turtlesim turtle_teleop_key

##  --------- Adding a Frame (Python) --------- 

#In our turtle example, we’ll add a new frame carrot1, which will be the child of the turtle1. This frame will serve as the goal for the second turtle
#Let’s first create the source files. Go to the learning_tf2_py package we created in the previous tutorials. Download the fixed frame broadcaster code by entering the following command:     
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/fixed_frame_tf2_broadcaster.py

#Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#Still in the root of your workspace, build your package:
colcon build --packages-select learning_tf2_py

#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#Now you are ready to run the launch file:
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py

#Therefore if we want our second turtle to follow the carrot instead of the first turtle, we need to change value of the target_frame. This can be done two ways. One way is to pass the target_frame argument to the launch file directly from the console:
ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1

#The extra frame we published in this tutorial is a fixed frame that doesn’t change over time in relation to the parent frame. However, if you want to publish a moving frame you can code the broadcaster to change the frame over time. 
#Let’s change our carrot1 frame so that it changes relative to turtle1 frame over time. Now download the dynamic frame broadcaster code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/dynamic_frame_tf2_broadcaster.py

#Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#Still in the root of your workspace, build your package:
colcon build --packages-select learning_tf2_py
 
#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash
  
#Now you are ready to run the launch file:
ros2 launch learning_tf2_py turtle_tf2_dynamic_frame_demo.launch.py


##  --------- Using Time (Python) --------- 

#In previous tutorials, we recreated the turtle demo by writing a tf2 broadcaster and a tf2 listener. We also learned how to add a new frame to the transformation tree. Now we will learn more about the timeout argument which makes the lookup_transform wait for the specified transform for up to the specified duration before throwing an exception.
#Edit turtle_tf2_listener.py and remove the timeout=Duration(seconds=1.0) parameter that is passed to the lookup_transform() call on line 76. It should look like shown below:
trans = self._tf_buffer.lookup_transform(
   to_frame_rel,
   from_frame_rel,
   now)
#Edit the exception handling on line 81 by adding newly imported exceptions and raise statement to see the exception:
except (LookupException, ConnectivityException, ExtrapolationException):
   self.get_logger().info('transform not ready')
   raise
   return
#If you now try to run the launch file, you will notice that it is failing:
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py

#edit your code on line 76 as shown below (return the timeout parameter):

trans = self._tf_buffer.lookup_transform(
   to_frame_rel,
   from_frame_rel,
   now,
   timeout=rclpy.duration.Duration(seconds=1.0))

#You can now run the launch file.
ros2 launch learning_tf2_py turtle_tf2_demo.launch.py


## ---------  Traveling in Time (Python) --------- 

#First, let’s go back to where we ended in the previous tutorial Using time. Go to your learning_tf2_py package.
#Now, instead of making the second turtle go to where the carrot is now, we will make the second turtle go to where the first carrot was 5 seconds ago. Edit the lookup_transform() call in turtle_tf2_listener.py file to

when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)
trans = self.tf_buffer.lookup_transform(
    to_frame_rel,
    from_frame_rel,
    when,
    timeout=rclpy.duration.Duration(seconds=0.05))

#Now if you run this, during the first 5 seconds, the second turtle would not know where to go because we do not yet have a 5-second history of poses of the carrot. But what happens after these 5 seconds? Let’s just give it a try:

ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py

#Let’s run the simulation again, this time with the advanced time-travel API:

ros2 launch learning_tf2_py turtle_tf2_fixed_frame_demo.launch.py

##  --------- Traveling in Time (C++) --------- 

#First, let’s go back to where we ended in the previous tutorial Using time. Go to your learning_tf2_cpp package.

#Now, instead of making the second turtle go to where the carrot is now, we will make the second turtle go to where the first carrot was 5 seconds ago. Edit the lookupTransform() call in turtle_tf2_listener.cpp file to

rclcpp::Time when = this->get_clock()->now() - rclcpp::Duration(5, 0);
t = tf_buffer_->lookupTransform(
    toFrameRel,
    fromFrameRel,
    when,
    50ms);

#Now if you run this, during the first 5 seconds, the second turtle would not know where to go because we do not yet have a 5-second history of poses of the carrot. But what happens after these 5 seconds? Let’s just give it a try:
ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py

#Let’s run the simulation again, this time with the advanced time-travel API:

ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py

##  --------- Debugging --------- 

#Go to the learning_tf2_cpp package we created in tf2 tutorials. Inside the src directory make a copy of the source file turtle_tf2_listener.cpp and rename it to turtle_tf2_listener_debug.cpp.
#Open the file using your preferred text editor, and change line 67 to

std::string toFrameRel = "turtle3";

#and change lookupTransform() call in lines 75-79 to

try {
   transformStamped = tf_buffer_->lookupTransform(
     toFrameRel,
     fromFrameRel,
     this->now());
} catch (tf2::TransformException & ex) {

#Now let’s run it to see what happens:
ros2 launch learning_tf2_cpp start_tf2_debug_demo.launch.py

#You will now see that the turtlesim came up. At the same time, if you run the turtle_teleop_key in another terminal window, you can use the arrow keys to drive the turtle1 around.
ros2 run turtlesim turtle_teleop_key

#Firstly, to find out if tf2 knows about our transform between turtle3 and turtle1, we will use tf2_echo tool.
ros2 run tf2_ros tf2_echo turtle3 turtle1

#Then what frames do exist? If you like to get a graphical representation of this, use view_frames tool.
ros2 run tf2_tools view_frames

#And now stop the running demo, build it, and run it again:
ros2 launch turtle_tf2 start_debug_demo.launch.py

#To get statistics on the timing, call tf2_monitor with corresponding frames.
ros2 run tf2_ros tf2_monitor turtle2 turtle1

##  --------- Using Stamped Datatypes with tf2_ros::MessageFilter --------- 

                     
#Go to the learning_tf2_py package we created in the previous tutorial. Inside the src/learning_tf2_py/learning_tf2_py directory download the example sensor message broadcaster code by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_message_broadcaster.py

#Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#And then we can build the package:
colcon build --packages-select learning_tf2_py

#Go to the learning_tf2_cpp package we created in the previous tutorial. Inside the src/learning_tf2_cpp/src directory download file turtle_tf2_message_filter.cpp by entering the following command:
wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_message_filter.cpp

#Run rosdep in the root of your workspace to check for missing dependencies.
rosdep install -i --from-path src --rosdistro humble -y

#Now open a new terminal, navigate to the root of your workspace, and rebuild the package with command:
 colcon build --packages-select learning_tf2_cpp
 
#Open a new terminal, navigate to the root of your workspace, and source the setup files:
. install/setup.bash

#First we need to run several nodes (including the broadcaster node of PointStamped messages) by launching the launch file turtle_tf2_sensor_message.launch.py:
ros2 launch learning_tf2_py turtle_tf2_sensor_message.launch.py

#This will bring up the turtlesim window with two turtles, where turtle3 is moving along a circle, while turtle1 isn’t moving at first. But you can run the turtle_teleop_key node in another terminal to drive turtle1 to move:
ros2 run turtlesim turtle_teleop_key
 
#Now if you echo the topic turtle3/turtle_point_stamped:
ros2 topic echo /turtle3/turtle_point_stamped

#When the demo is running, open another terminal and run the message filter/listener node:
ros2 run learning_tf2_cpp turtle_tf2_message_filter



#  --------- ROS 2 Testing Tutorials --------- 

#Perhaps the most important benefit of writing tests is that tests make you a good citizen. Tests influence quality in the long term. It is a well accepted practice in many open-source projects. By writing regressions tests, you are contributing to long term quality of the ROS ecosystem.


##  --------- Running Tests in ROS 2 from the Command Line --------- 

#Build and run your tests
#To compile and run the tests, simply run the test verb from colcon.
colcon test --ctest-args tests [package_selection_args]

#To see the results, simply run the test-result verb from colcon.
colcon test-result --all

#To see the exact test cases which fail, use the --verbose flag:
colcon test-result --all --verbose

#Since the previous build type may be cached by CMake, clean the cache and rebuild.
colcon build --cmake-clean-cache --mixin debug

#Next, run the test directly through gdb. For example:
gdb -ex run ./build/rcl/test/test_logging


##  --------- Writing Basic Tests with C++ using GTest --------- 

#Starting point: we’ll assume you have a basic ament_cmake package set up already and you want to add some tests to it.
#In this tutorial, we’ll be using gtest.

#We’ll start off with our code in a file called test/tutorial_test.cpp
#include <gtest/gtest.h>

TEST(package_name, a_first_test)
{
  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#Add the following line to package.xml

<test_depend>ament_cmake_gtest</test_depend>

##  --------- Writing Basic Tests with Python --------- 
                       
#Beyond the standard colcon testing commands you can also specify arguments to the pytest framework from the command line with the --pytest-args flag. For example, you can specify the name of the function to run with
colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function

#To see the pytest output while running the tests, use these flags:
colcon test --event-handlers console_cohesion+

#  --------- ROS 2 URDF Tutorials --------- 

#URDF (Unified Robot Description Format) is a file format for specifying the geometry and organization of robots in ROS.

##  --------- Building a Visual Robot Model from Scratch --------- 

#To examine the model, launch the display.launch.py file:
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf

#A slightly modified argument allows this to work regardless of the current working directory:
ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf

#The joint is defined in terms of a parent and a child. URDF is ultimately a tree structure with one root link. This means that the leg’s position is dependent on the base_link’s position.
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf

#Now, looking at the leg’s visual origin, it has both a xyz and rpy offset. This defines where the center of the visual element should be, relative to its origin. Since we want the leg to attach at the top, we offset the origin down by setting the z offset to be -0.3 meters.
#And since we want the long part of the leg to be parallel to the z axis, we rotate the visual part PI/2 around the Y axis.
ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf

#You can also use a texture to specify an image file to be used for coloring the object
ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf

ros2 launch urdf_tutorial display.launch.py model:=urdf/05-visual.urdf
                               

##  --------- Building a Movable Robot Model --------- 

                   
#To visualize and control this model, run the same command as the last tutorial:
ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf

##  --------- Adding physical and collision properties ---------

#Goal: Learn how to add collision and inertial properties to links, and how to add joint dynamics to joints.

##  --------- Using Xacro to Clean Up Your Code --------- 

#Goal: Learn some tricks to reduce the amount of code in a URDF file using Xacro

#As its name implies, xacro is a macro language for XML. The xacro program runs all of the macros and outputs the result. Typical usage looks something like this:
xacro model.xacro > model.urdf

#To see the model generated by a xacro file, run the same command as with previous tutorials:
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro

                     

##  --------- Using URDF with robot_state_publisher --------- 

        
#As always, don’t forget to source ROS 2 in every new terminal you open. Create your package:
mkdir -p ~/second_ros2_ws/src
cd ~/second_ros2_ws/src
ros2 pkg create urdf_tutorial_r2d2 --build-type ament_python --dependencies rclpy --license Apache-2.0
cd urdf_tutorial_r2d2

#Create the directory where we will store some assets:
mkdir -p urdf

#Install the packages:
cd ~/second_ros2_ws
colcon build --symlink-install --packages-select urdf_tutorial_r2d2
source install/setup.bash

#Launch the package
ros2 launch urdf_tutorial_r2d2 demo.launch.py

#Open a new terminal, the run Rviz using
rviz2 -d ~/second_ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
