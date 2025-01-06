import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Pose2D
import math

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.init_variables()
        self.init_subscribers()
        self.init_publisher()

    def init_variables(self):
        self.goals = [
            (7.0, 7.0, 0.0),
            (3.0, 7.0, 0.0),
            (3.0, 3.0, 0.0),
            (7.0, 3.0, 0.0),
            (5.5, 5.5, 0.0)
        ]
        self.current_goal_index = 0
        self.x = 0.0
        self.y = 0.0
        self.tolerance = 0.01  

    def init_publisher(self):
        self.goal_publisher = self.create_publisher(Pose2D, '/mbp/goal', 10)
        self.timer = self.create_timer(1.0, self.check_and_publish_goal)

    def init_subscribers(self):
        self.create_subscription(Pose, '/mbp/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y

    def check_and_publish_goal(self):
        if self.current_goal_index < len(self.goals):
            goal_x, goal_y, goal_theta = self.goals[self.current_goal_index]
            distance_to_goal = math.sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)
            
            if distance_to_goal < self.tolerance:
                self.get_logger().info(f'Objetivo {self.current_goal_index + 1} alcançado!')
                self.current_goal_index += 1    

            if self.current_goal_index < len(self.goals):
                goal_x, goal_y, goal_theta = self.goals[self.current_goal_index]
                goal_msg = Pose2D()
                goal_msg.x = goal_x
                goal_msg.y = goal_y
                goal_msg.theta = goal_theta
                self.goal_publisher.publish(goal_msg)
                self.get_logger().info(f'Novo objetivo: x={goal_x}, y={goal_y}, theta={goal_theta}')
            else:
                self.get_logger().info('Todos os objetivos foram alcançados!')

def main(args=None):
    rclpy.init(args=args)
    goal_manager = GoalManager()
    rclpy.spin(goal_manager)
    goal_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
