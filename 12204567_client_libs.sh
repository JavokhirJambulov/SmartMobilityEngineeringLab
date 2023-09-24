#-------------Using colcon to build packages-------------

#Install colcon
sudo apt install python3-colcon-common-extensions

#First, create a directory (ros2_ws) to contain our workspace:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

#Let’s clone the examples repository into the src directory of the workspace:
git clone https://github.com/ros2/examples src/examples -b humble

colcon build --symlink-install
#To run tests for the packages we just built, run the following:
colcon test

#Source the environment
source install/setup.bash

#Let’s run a subscriber node from the examples:
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function

#In another terminal, let’s run a publisher node (don’t forget to source the setup script):
ros2 run examples_rclcpp_minimal_publisher publisher_member_function

#The command colcon_cd allows you to quickly change the current working directory of your shell to the directory of a package. As an example colcon_cd some_ros_package would quickly bring you to the directory ~/ros2_ws/src/some_ros_package.
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

#The command colcon supports command completion for bash and bash-like shells if the colcon-argcomplete package is installed.
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc


#-------------Creating a workspace-------------

source /opt/ros/humble/setup.bash

#Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

#In the ros2_ws/src directory, run the following command:
git clone https://github.com/ros/ros_tutorials.git -b humble
     
#From the root of your workspace (ros2_ws), run the following command:
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y

#From the root of your workspace (ros2_ws), you can now build your packages using the command:
colcon build

#  In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:
 source /opt/ros/humble/setup.bash
  
#Go into the root of your workspace:
cd ~/ros2_ws

#In the root, source your overlay:                                             
source install/local_setup.bash

#Now you can run the turtlesim package from the overlay:
ros2 run turtlesim turtlesim_node

#Return to the second terminal (where the overlay is sourced) and run turtlesim again:
ros2 run turtlesim turtlesim_node

#To see that your underlay is still intact, open a brand new terminal and source only your ROS 2 installation. Run turtlesim again:
ros2 run turtlesim turtlesim_node




#-------------Creating a package-------------

                                      
#Make sure you are in the src folder before running the package creation command.
cd ~/ros2_ws/src

#Enter the following command in your terminal:
ros2 pkg create --build-type ament_cmake --node-name my_node my_package   

#Return to the root of your workspace:
cd ~/ros2_ws

#Now you can build your packages:
colcon build    

#To build only the my_package package next time, you can run:
colcon build --packages-select my_package  

#Then, from inside the ros2_ws directory, run the following command to source your workspace:
source install/local_setup.bash

#To run the executable you created using the --node-name argument during package creation, enter the command:
ros2 run my_package my_node       

#From ros2_ws/src/my_package, open package.xml using your preferred text editor:
#Input your name and email on the maintainer line if it hasn’t been automatically populated for you. Then, edit the description line to summarize the package:
<description>Beginner client libraries tutorials practice package</description>

#Then, update the license line. You can read more about open source licenses here. Since this package is only for practice, it’s safe to use any license. We’ll use Apache License 2.0:
<license>Apache License 2.0</license>


#-------Writing a simple publisher and subscriber (C++)-------------

         
#1: Create a package
# Source your ROS 2 installation
source /path/to/ros2/install/setup.bash

# Navigate to your ROS 2 workspace directory
cd /path/to/ros2_ws

# navigate into ros2_ws/src, and run the package creation command:
ros2 pkg create --build-type ament_cmake cpp_pubsub

# 2: Write the publisher node
# Download the example talker code
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp

# Open publisher_member_function.cpp and paste the code from Ros doc site

# Update package.xml with description, maintainer, and license information

# Update CMakeLists.txt to add the talker executable

# Build the package
colcon build --packages-select cpp_pubsub

#3: Write the subscriber node
# Download the example subscriber code
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp

# Open subscriber_member_function.cpp and paste the code from Ros doc site

# Update CMakeLists.txt to add the executable

# Build the package again
colcon build --packages-select cpp_pubsub

# 4: Run the publisher node
# Open a new terminal and navigate to the workspace
cd /path/to/ros2_ws
source install/setup.bash

# Run the talker node
ros2 run cpp_pubsub talker

# The terminal should start publishing messages

# Open another terminal, navigate to the workspace, and source the setup files
cd /path/to/ros2_ws
source install/setup.bash

# Start the listener node
ros2 run cpp_pubsub listener

# The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time.

# Enter Ctrl+C in each terminal to stop the nodes from spinning.




#-------Writing a simple publisher and subscriber (Python)--------

                      
# 1: Create a package
# Source your ROS 2 installation
source /path/to/ros2/install/setup.bash

# navigate into ros2_ws/src, and run the package creation command:
cd /path/to/ros2_ws
ros2 pkg create --build-type ament_python py_pubsub

# 2: Write the publisher node
# Navigate to the directory of the Python package
cd /path/to/ros2_ws/src/py_pubsub/py_pubsub

# Download the example talker code
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

# Open publisher_member_function.py and paste the code

# Update package.xml with description, maintainer, and license information

# Update setup.py to add an entry point for the talker executable

# Build the package
colcon build --packages-select py_pubsub

# 3: Write the subscriber node
# Navigate to the directory of the Python package
cd /path/to/ros2_ws/src/py_pubsub/py_pubsub

# Download the example subscriber code
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

# Open subscriber_member_function.py and paste the code

# Update setup.py to add an entry point for the listener executable

# Build the package again
colcon build --packages-select py_pubsub

# 4: Run the publisher node
# Open a new terminal and navigate to the workspace
cd /path/to/ros2_ws
source install/setup.bash

# Run the talker node
ros2 run py_pubsub talker

# The terminal should start publishing messages

# Open another terminal, navigate to the workspace, and source the setup files
cd /path/to/ros2_ws
source install/setup.bash

# Run the listener node
ros2 run py_pubsub listener

# The listener will start printing received messages

# To stop the nodes, press Ctrl+C in their respective terminals




#-------------Writing a simple service and client (C++)-------------

                      
# Source your ROS 2 installation
source /path/to/ros2/install/setup.bash

# Navigate into ros2_ws/src and create a new package:
cd /path/to/ros2_ws
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces

# Inside the ros2_ws/src/cpp_srvcli/src directory, create a new file called add_two_ints_server.cpp and paste the following code within:

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}


# Inside the ros2_ws/src/cpp_srvcli/src directory, create a new file called add_two_ints_client.cpp and paste the following code within:
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}

# Add the following lines to CMakeLists.txt to create an executable named server:
# add_executable(server src/add_two_ints_server.cpp)

# Build the ROS 2 package
colcon build --packages-select cpp_srvcli

# Source the setup files
source install/setup.bash

# Run the service node
ros2 run cpp_srvcli server

# Open another terminal and source the setup files
# Run the client node with two integers as arguments (e.g., 2 and 3)
ros2 run cpp_srvcli client 2 3

# The client will receive a response, and the server will log messages
# Enter Ctrl+C in the server terminal to stop the node from spinning




#-------------Writing a simple service and client (Python)-------------

# Navigate to your ROS 2 workspace directory
cd /path/to/your/ros2_ws

# Create a new ROS 2 package for the service and client
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces

# As always, though, make sure to add the description, maintainer email and name, and license information to package.xml.
# Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called service_member_function.py and paste the following code within:

# To allow the ros2 run command to run your node, you must add the entry point to setup.py (located in the ros2_ws/src/py_srvcli directory).
# Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called client_member_function.py and paste the following code within:


# Build your ROS 2 package
colcon build --packages-select py_srvcli

# Source the ROS 2 setup files
source install/setup.bash

# Run the service node
ros2 run py_srvcli service

# In a new terminal, source the ROS 2 setup files
source install/setup.bash

# Open another terminal and source the setup files from inside ros2_ws again. Start the client node, followed by any two integers separated by a space:
ros2 run py_srvcli client 2 3
        

#-------------Creating custom msg and srv files-------------

           
# Create a new package
ros2 pkg create --build-type ament_cmake tutorial_interfaces

# Create custom message and service definitions in the corresponding directories as mentioned in the tutorial.

#To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages.

#Add the following lines within the <package> element of package.xml:
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

# In the root of your workspace (~/ros2_ws), run the following command:
colcon build --packages-select tutorial_interfaces

# Confirm message and service creation
#In a new terminal, run the following command from within your workspace (ros2_ws) to source it:
source install/setup.bash
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/msg/Sphere
ros2 interface show tutorial_interfaces/srv/AddThreeInts

# Test the new interfaces using your packages created in previous tutorials.
# Make necessary modifications to the publisher/subscriber and service/client packages.

# Build and run your packages accordingly.




#-------------Implementing custom interfaces-------------

          
# Create a new package named more_interfaces and create a directory for msg files
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg

# Create a msg file named AddressBook.msg and define the message structure
echo "uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type" > more_interfaces/msg/AddressBook.msg

# Edit package.xml to add build and runtime dependencies and specify the package group
# Add these lines within the <package> element:
# <buildtool_depend>rosidl_default_generators</buildtool_depend>
# <exec_depend>rosidl_default_runtime</exec_depend>
# <member_of_group>rosidl_interface_packages</member_of_group>

# Open CMakeLists.txt and add the following lines:
# Find the package that generates message code from msg/srv files:
# find_package(rosidl_default_generators REQUIRED)

# Declare the list of messages you want to generate:
# set(msg_files
#   "msg/AddressBook.msg"
# )

# Generate the messages:
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
# )

# Export the message runtime dependency:
# ament_export_dependencies(rosidl_default_runtime)


#Return to the root of the workspace to build the package:

cd ~/ros2_ws
colcon build --packages-up-to more_interfaces

#Then source the workspace and run the publisher:

source install/local_setup.bash
ros2 run more_interfaces publish_address_book


#-------------Using parameters in a class (C++)-------------


#Recall that packages should be created in the src directory, not the root of the workspace. Navigate into ros2_ws/src and create a new package:
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp

#it’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro humble -y

#Navigate back to the root of your workspace, ros2_ws, and build your new package:
colcon build --packages-select cpp_parameters

#Open a new terminal, navigate to ros2_ws, and source the setup files:
source install/setup.bash
ros2 run cpp_parameters minimal_param_node


#Open another terminal, source the setup files from inside ros2_ws again, and enter the following line:
ros2 param list

#There you will see the custom parameter my_parameter. To change it, simply run the following line in the console:
ros2 param set /minimal_param_node my_parameter earth

#Open a console and navigate to the root of your workspace, ros2_ws, and build your new package:
colcon build --packages-select cpp_parameters

#Then source the setup files in a new terminal:
source install/setup.bash

#Now run the node using the launch file we have just created:
ros2 launch cpp_parameters cpp_parameters_launch.py

#The terminal should return the following message every second:
#[INFO] [custom_minimal_param_node]: Hello earth!





#-------------Using parameters in a class (Python)-------------


#Recall that packages should be created in the src directory, not the root of the workspace. Navigate into ros2_ws/src and create a new package:
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy

#As always, though, make sure to add the description, maintainer email and name, and license information to package.xml.

#It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
rosdep install -i --from-path src --rosdistro humble -y

#Navigate back to the root of your workspace, ros2_ws, and build your new package:
colcon build --packages-select python_parameters

#Open a new terminal, navigate to ros2_ws, and source the setup files:
source install/setup.bash

#Now run the node:
ros2 run python_parameters minimal_param_node

#The terminal should return the following message every second:
#[INFO] [parameter_node]: Hello world!

#Make sure the node is running:
ros2 run python_parameters minimal_param_node

#Open another terminal, source the setup files from inside ros2_ws again, and enter the following line:
ros2 param list

#There you will see the custom parameter my_parameter. To change it, simply run the following line in the console:
ros2 param set /minimal_param_node my_parameter earth

#Open a console and navigate to the root of your workspace, ros2_ws, and build your new package:
colcon build --packages-select python_parameters

#Then source the setup files in a new terminal:
source install/setup.bash

#Now run the node using the launch file we have just created:
ros2 launch python_parameters python_parameters_launch.py

#The terminal should return the following message every second:

#[INFO] [custom_minimal_param_node]: Hello earth!                          





#-------------Using ros2doctor to identify issues-------------

        
# To examine your general ROS 2 setup as a whole with ros2doctor. First, source ROS 2 in a new terminal, then enter the command:
ros2 doctor

#Check a system
#Start up the system by opening a new terminal, sourcing ROS 2, and entering the command:
ros2 run turtlesim turtlesim_node

#Open another terminal:
ros2 run turtlesim turtle_teleop_key

#You can try this by opening two new terminals while turtlesim is still running, sourcing ROS 2 in each, and running each of the following commands in their own terminal:
ros2 topic echo /turtle1/color_sensor
ros2 topic echo /turtle1/pose

#Get a full report
#To get a full report, enter the following command in the terminal:
ros2 doctor --report





#-------------Creating and using plugins (C++)-------------


                                   
#Create a new empty package in your ros2_ws/src folder with the following command:
ros2 pkg create --build-type ament_cmake --dependencies pluginlib --node-name area_node --license Apache-2.0 polygon_base

#Navigate back to the root of your workspace, ros2_ws, and build your new packages:
colcon build --packages-select polygon_base polygon_plugins

#From ros2_ws, be sure to source the setup files:
source install/setup.bash

#Now run the node:
ros2 run polygon_base area_node

#It should print:
#Triangle area: 43.30
#Square area: 100.00



