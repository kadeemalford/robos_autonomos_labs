import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D
import math


class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.k_linear = 1.0
        self.k_angular = 6.0
        self.error_tolerance = 0.1

    def init_publisher(self):
        self.publisher = self.create_publisher(Twist, '/mbp/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.pub_callback)

    def init_subscribers(self):
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_theta = msg.theta

    def pub_callback(self):
        linear_error = math.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
        angular_error = math.atan2(self.goal_y - self.y, self.goal_x - self.x) - self.theta
        linear_velocity = self.k_linear * math.tanh(linear_error)
        angular_velocity = self.k_angular * angular_error

        if linear_error < self.error_tolerance:
            linear_velocity = 0.0
            angular_velocity = 0.0

        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    turtle_control = TurtleControl()
    rclpy.spin(turtle_control)
    turtle_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()