import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/robot_effort_controller/commands',
            10
        )
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Float64MultiArray()
        # 为四个轮子设置目标力
        msg.data = [0.01, 0.01, 0.01, 0.01]  # 示例值
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()