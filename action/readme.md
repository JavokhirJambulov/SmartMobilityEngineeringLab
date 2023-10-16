12204567 Javokhir Action Creation

# Project Title: Action-Based Obstacle Avoidance with ROS 2 and Turtlesim


## Introduction:

The project aims to demonstrate action-based obstacle avoidance using the Robot Operating System 2 (ROS 2) framework in a simulated environment provided by turtlesim. The primary objective is to create an action server that controls the movement of a virtual "TurtleBot" while avoiding obstacles and an action client to send movement commands.

## Project Components:

### Action Server (Turtle Action Server):

Responsible for handling action goals and executing movements in the turtlesim environment.
Implements obstacle avoidance logic to ensure safe navigation.
Provides feedback to the client regarding the progress of the action.
Sends a result indicating the outcome of the action (e.g., success or failure).

### Action Client (MoveTurtle Client):

Sends action goals to the Turtle Action Server.
Monitors the execution of the action and receives feedback from the server.
Displays the result of the action execution.

### Turtlesim Environment:

Provides a virtual simulation environment in which the "TurtleBot" operates.
Accepts velocity commands via topics for controlling the "TurtleBot."
Serves as the context for the action-based obstacle avoidance demonstration.

# How to run the code:

To run your ROS 2 code that includes the MoveTurtle action server and client, follow these steps:

# Source Your ROS 2 Workspace:

Before you run your code, you need to source your ROS 2 workspace. Replace <your_ros2_workspace> with the actual path to your ROS 2 workspace.

source <your_ros2_workspace>/install/setup.bash

# Run the Action Server:

Open a new terminal and run your action server. Replace <your_package_name> with the actual name of your ROS 2 package:

ros2 run <your_package_name> move_turtle_server

This will start the action server, which is responsible for moving the virtual "TurtleBot" in turtlesim.

# Run the Action Client:

Open another terminal and run your action client. Replace <your_package_name> with the actual name of your ROS 2 package:

ros2 run <your_package_name> move_turtle_client
This will send a goal to the action server, instructing the virtual "TurtleBot" to move to a specified position in turtlesim.

# Observe Execution:

You should now see the virtual "TurtleBot" in turtlesim move to the goal position based on the client's goal input. The action client will also print the result of the action, indicating whether it was successful or not.

Make sure that you have the turtlesim GUI or terminal interface running to visualize the movement of the "TurtleBot" in turtlesim.

Keep in mind that this is a simplified example, and in a real-world scenario, the goal for the "TurtleBot" would typically come from higher-level planning or sensor data. Additionally, you may need to customize the action server logic for obstacle avoidance and advanced navigation as per your specific requirements.
