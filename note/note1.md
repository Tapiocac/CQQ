
- 完成建模（基座+驱动轮+机械臂底座+大臂小臂）
- 完成ros2自有力控制器控制


四轮小车+机械臂

- 本体 0.20 0.15 0.05
- 四个驱动轮
- 机械臂

`ros2 topic pub /robot_effort_controller/commands std_msgs/msg/Float64MultiArray "{data:[0.1,0.1,0.1,0.1]}"`

`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

1. 编写ros2_control硬件资源,使用Gazebo插件解析ros2_control标签
2. 编写1中Gazebo插件参数文件
3. 在robot.xacro中引入ros2_control硬件资源
4. 在参数文件中添加关节状态发布器，并在launch文件中运行
5. 在参数文件中添加力控制器，并在launch文件中运行


