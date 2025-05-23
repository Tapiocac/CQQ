import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller_new')
        
        # 订阅目标速度
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 订阅真实速度
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 发布轮子力矩命令
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/robot_effort_controller/commands', 10)
        
        # 小车参数
        self.wheel_radius = 0.03  # 轮子半径 (m)
        self.wheel_separation = 0.14  # 轮距 (m)
        
        # PID 参数
        self.Kp = 10.0
        self.Ki = 5.0
        self.Kd = 1.0
        
        # 存储当前轮速（用于 PID 计算）
        self.current_wheel_speeds = {'left1_wheel_joint':0,'right1_wheel_joint':0}  

    def joint_state_callback(self, msg):
        """ 更新当前轮速 """
        for i, name in enumerate(msg.name):
            if name in ['left1_wheel_joint', 'right1_wheel_joint']:
                self.current_wheel_speeds[name] = msg.velocity[i]
        
    def cmd_vel_callback(self, msg):
        """ 计算目标轮速并发送力矩命令 """
        v_desired = msg.linear.x  # 线速度 (m/s)
        ω_desired = msg.angular.z  # 角速度 (rad/s)
        
        # 计算目标轮速（差速模型）
        target_FL = (v_desired - ω_desired * self.wheel_separation / 2) / self.wheel_radius
        target_FR = (v_desired + ω_desired * self.wheel_separation / 2) / self.wheel_radius
        target_RL = target_FL  # 四轮小车通常前后轮速度相同
        target_RR = target_FR
        
        # 计算 PID 输出力矩
        effort_FL = self.compute_pid_effort(target_FL, self.current_wheel_speeds['left1_wheel_joint'])
        effort_FR = self.compute_pid_effort(target_FR, self.current_wheel_speeds['right1_wheel_joint'])
        effort_RL = effort_FL
        effort_RR = effort_FR
        
        # 发布力矩命令
        effort_msg = Float64MultiArray()
        effort_msg.data = [effort_FL, effort_FR, effort_RL, effort_RR]
        self.effort_pub.publish(effort_msg)
    
    def compute_pid_effort(self, target_speed, current_speed):
        """ 计算 PID 力矩 """
        error = target_speed - current_speed
        # 简单 PID 计算
        effort = self.Kp * error
        return effort

def main():
    rclpy.init()
    node = WheelController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()