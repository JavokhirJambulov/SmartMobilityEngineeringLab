ID: 12204567

Name: Javokhir Jambulov


# Fleet Management ROS 2 Application

This ROS 2 application, Fleet Management, efficiently allocates and routes vehicles for smart mobility services. This README provides instructions for setting up and running the application. Please follow the steps below.

<img width="379" alt="Annotation 2023-10-22 000701" src="https://github.com/JavokhirJambulov/SmartMobilityEngineeringLab/assets/91411930/a4ae3a5a-198f-4ad0-8fcf-4aa894cb0173">

<img width="370" alt="Annotation 2023-10-22 000724" src="https://github.com/JavokhirJambulov/SmartMobilityEngineeringLab/assets/91411930/97055163-3775-4786-b721-a66945116abf">

## Prerequisites

- ROS 2 installed (Foxy or later)
- Python 3.6 or later
- Your ROS 2 workspace set up (e.g., ~/ros2_ws)

## Installation

1. Clone this repository into your ROS 2 workspace's source directory:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/yourusername/fleet_management_package.git
    ```

2. Build your ROS 2 workspace to compile the package:

    ```bash
    cd ~/ros2_ws
    colcon build
    ```

## Running the ROS 2 Application

### Launching ROS 2 Master

1. Open a terminal and source your ROS 2 workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Start the ROS 2 master:

    ```bash
    ros2 run ros_core ros2_control_node
    ```

### Running the Action Server

To run the Fleet Management Action Server:

1. Open a new terminal and source your ROS 2 workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the Action Server:

    ```bash
    ros2 run your_package_name fleet_management_server_cli.py
    ```

### Running the Action Client

To run the Fleet Management Action Client:

1. Open a new terminal and source your ROS 2 workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the Action Client, and enter the fleet size when prompted:

    ```bash
    ros2 run your_package_name fleet_management_client_cli.py
    ```

## Professional CLI Usage

- The Action Client (`fleet_management_client_cli.py`) will prompt you to input the fleet size, and it will send the request to the Action Server.
- The Action Server (`fleet_management_server_cli.py`) processes the request, allocates and routes vehicles, and provides feedback on completion percentage.
- The feedback is displayed in the Action Client terminal.

## Testing Scenarios

- **Scenario 1:** Testing with Various Fleet Sizes
  - Run the Action Server.
  - Run the Action Client multiple times with different fleet sizes to observe the allocation and routing results.

- **Scenario 2:** Error Handling
  - Test how the application handles incorrect inputs, such as non-integer fleet sizes or negative values.



## Additional Information

- Detailed documentation for the Fleet Management ROS 2 application can be found in the `docs` directory of the package.

- For more information on ROS 2 and ROS 2 packages, refer to the official [ROS 2 documentation](https://docs.ros.org/en/humble/).
