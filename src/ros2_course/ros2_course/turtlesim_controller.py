import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleSimController(Node):
    def __init__(self, linear_speed=2.0, angular_speed=2.0):
        super().__init__('turtlesim_controller')
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.update_pose, 10)
        self.pose = None
        self.get_logger().info('Waiting for turtle pose...')
        while self.pose is None:
            rclpy.spin_once(self)

    def update_pose(self, msg):
        self.pose = msg
        if self.pose:
            self.get_logger().info(f'Pose updated: x={self.pose.x}, y={self.pose.y}, theta={self.pose.theta}')

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear) * self.linear_speed
        msg.angular.z = float(angular) * self.angular_speed
        self.publisher_.publish(msg)

    def turn(self, radians, tolerance=0.01):
        target_theta = (self.pose.theta + radians) % (2 * math.pi)
        error = self.normalize_angle(target_theta - self.pose.theta)
        speed = 0.5 * math.copysign(1, error)
        while abs(error) > tolerance:
            rclpy.spin_once(self)
            error = self.normalize_angle(target_theta - self.pose.theta)
            self.send_velocity(0.0, speed)
        self.send_velocity(0.0, 0.0)
        self.get_logger().info(f'Turned by {radians} radians.')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def go_straight(self, distance, tolerance=0.1):
        start_pose = self.pose
        self.send_velocity(1.0, 0.0)
        while self.distance(self.pose, start_pose) < distance:
            rclpy.spin_once(self)
        self.send_velocity(0.0, 0.0)
        self.get_logger().info(f'Moved straight by {distance} units.')

    def distance(self, pose1, pose2):
        return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)

    def draw_koch_side(self, length, level):
        if level == 0:
            self.go_straight(length)
        else:
            next_length = length / 3.0
            self.draw_koch_side(next_length, level - 1)
            self.turn(math.pi / 3)
            self.draw_koch_side(next_length, level - 1)
            self.turn(-2 * math.pi / 3)
            self.draw_koch_side(next_length, level - 1)
            self.turn(math.pi / 3)
            self.draw_koch_side(next_length, level - 1)

    def create_snowflake(self, length, level):
        for _ in range(3):
            self.draw_koch_side(length, level)
            self.turn(-2 * math.pi / 3)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleSimController(linear_speed=2.0, angular_speed=2.0)
    # Increase the recursion level here for more detail
    turtle_controller.create_snowflake(5.0, 4)  # Adjust length and level for desired detail
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
