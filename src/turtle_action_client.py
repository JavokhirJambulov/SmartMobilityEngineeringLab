import rclpy
from rclpy.action import ActionClient
from obstacle_avoidance.action import MoveTurtle
from turtlesim.msg import Pose

def main(args=None):
    rclpy.init(args=args)
    client = ActionClient('move_turtle', MoveTurtle)

    # Wait for the action server to be ready
    if not client.wait_for_server(timeout_sec=4.0):
        rclpy.get_logger().error('Action server not available')
        return

    # Create a goal
    goal_msg = MoveTurtle.Goal()
    
    # Define the goal pose (x, y) where you want the "TurtleBot" to move
    goal_pose = Pose()
    goal_pose.x = 5.0  # Adjust the desired x-coordinate
    goal_pose.y = 5.0  # Adjust the desired y-coordinate
    goal_msg.goal = goal_pose

    # Send the goal to the action server
    future = client.send_goal_async(goal_msg)

    # Monitor goal execution
    while rclpy.ok():
        if future.done():
            if future.result() is not None:
                result = future.result().result
                rclpy.get_logger().info(f'Goal result: {result}')
            else:
                rclpy.get_logger().warning('Goal was canceled.')
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()

