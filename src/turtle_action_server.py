import rclpy
from rclpy.action import ActionServer
from your_package_name.msg import MoveTurtle
from nav2_msgs.msg import Odometry
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist

class MoveTurtleServer:
    def __init__(self):
        self.action_server = ActionServer(
            self,
            MoveTurtle,
            'move_turtle',
            self.execute_callback
        )

        self.feedback = Odometry()
        self.pose = Pose()

    def execute_callback(self, goal_handle):
        # Initialize feedback
        self.feedback = Odometry()

        # Assume the goal pose comes in the form of turtlesim/Pose
        goal_pose = goal_handle.request.goal

        # You can now implement the movement and obstacle avoidance logic
        # For simplicity, let's just move to the goal position

        # Calculate the distance to the goal
        distance_to_goal = abs(self.pose.x - goal_pose.x) + abs(self.pose.y - goal_pose.y)

        # Simulate movement to the goal
        while distance_to_goal > 0.01:
            # Calculate linear and angular velocity commands
            linear_vel = 1.0  # Adjust as needed
            angular_vel = 0.0

            # Update feedback
            self.feedback.pose.pose.position.x = self.pose.x
            self.feedback.pose.pose.position.y = self.pose.y

            # Simulate movement
            self.publisher.publish(Twist(linear=linear_vel, angular=angular_vel))
            rclpy.spin_once(self)

            # Calculate the updated distance to the goal
            distance_to_goal = abs(self.pose.x - goal_pose.x) + abs(self.pose.y - goal_pose.y)

        # You can provide more advanced obstacle avoidance logic here
        # Update feedback with the final position
        self.feedback.pose.pose.position.x = self.pose.x
        self.feedback.pose.pose.position.y = self.pose.y

        # Indicate action success and set result
        goal_handle.succeed(self.feedback)

def main(args=None):
    rclpy.init(args=args)
    server = MoveTurtleServer()
    rclpy.spin(server)

if __name__ == '__main__':
    main()

