# position_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import tf_transformations

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 设定目标位置
        self.goal_x = 2.0
        self.goal_y = 1.0

        self.kp = 1.0
        self.kp_theta = 2.0

    def odom_callback(self, msg):

        # 获取当前位置信息
        self.print_position(msg)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.get_yaw_from_quat(msg.pose.pose.orientation)

        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        dtheta = target_theta - theta

        cmd = Twist()
        cmd.linear.x = self.kp * distance
        cmd.angular.z = self.kp_theta * dtheta
        self.cmd_pub.publish(cmd)

    def get_yaw_from_quat(self, q):
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)
        return yaw
    
    # 输出当前位置信息
    def print_position(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.get_yaw_from_quat(msg.pose.pose.orientation)
        self.get_logger().info(f'Current Position: x={x}, y={y}, theta={theta}')

rclpy.init()
node = PositionController()
rclpy.spin(node)
