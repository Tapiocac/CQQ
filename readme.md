用于测试基于ros2_control的控制器开发

环境配置：
VSCode包含路径中添加`/opt/ros/humble/include/**`


必要安装：

ROS2 humble
鱼香肉丝一键安装

ros-humble-rqt-tf-tree
`sudo apt install ros-humble-rqt-tf-tree`
`rm -rf ~/.config/ros.org/rqt_gui.ini`

gazebo
`sudo apt install gazebo`

xacro
`sudo apt install ros-$ROS_DISTRO-xacro`

桥梁包
`sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs`

ros2_control
`sudo apt install ros-$ROS_DISTRO-ros2-control`

ros2_control自带控制器
`sudo apt install ros-$ROS_DISTRO-ros2-controllers`

Gazebo的ros2_control插件
`sudo apt install ros-$ROS_DISTRO-gazebo-ros2-control`

ros_team_workspace
`git clone ros_team_workspace`



tf-transformations四元数变换库


步骤：
1. 实现基于位置的控制


控制器函数：

# 构造函数 
调用基类`ControllerInterface`的构造函数

# `on_init`初始化

初始化参数监听器`param_listener_`与参数结构体`params_`

return: SUCCESS

# `command_interface_configuration`命令接口配置

指定控制器需要的命令接口（轮子速度接口）
返回：{type,names}
type:INDIVIDUAL 
names：接口完整名，例如"joint_name/velocity"

# `state_interface_configuration`状态接口配置

指定控制器需要的状态接口（轮子速度接口）
返回：{type,names}
type:INDIVIDUAL 
names：接口完整名，例如"joint_name/velocity"

# `on_configure`配置阶段

检查参数合法性，初始化订阅器、发布器、里程计等。
设置初始消息和相关参数。

- 更新参数，并判断参数合法性
- 进行重置
- 初始化速度指令和历史命令队列
- 创建并配置速度指令订阅器
- 创建并初始化里程计发布器和 TF 发布器及其消息内容
- 里程计初始化

return: SUCCESS

# `on_activate`激活阶段

注册左右轮的接口句柄，激活订阅和发布。




# 其他

std::ref 让你可以间接在容器中存放引用，并通过 .get() 访问原始对象，是现代C++中常用的工具。


input_ref_通过订阅话题接收上位机的命令，ros2_control根据yaml文件中的配置声明来生成参数结构体`Params`与接收和下发到硬件的接口`command_interfaces_`和`state_interfaces_`

`Params` 通过读取机器人描述文件中的`xxx_ros2_controller.yaml`配置文件得到信息

`command_interfaces_`和`state_interfaces_` 分别通过`set_value()`和`get_value()`来与硬件层传递信息，其中排列方式为：`joint1/interface1`,`joint1/interface2`,...,`joint4/interface4`



```cpp
# my_controller_parameters.hpp
struct Params {
    std::vector<std::string> joints = {};
    std::vector<std::string> state_joints = {};
    std::string command_interfaces = "";
    std::vector<std::string> state_interfaces = {};
    // for detecting if the parameter struct has been updated
    rclcpp::Time __stamp;
};
```

```cpp
# controller_interface_base.hpp
std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
```

## 自定义硬件层

info_ 信息来自于机器人ros2_control.xacro描述文件，包含了关节、传感器，以及其拥有的状态，命令接口等信息






## 测试部分
修改CMakeList，删除`if(BUILD_TESTING)`部分与`ament_cmake_gmock`,`ros2_control_test_assets`包依赖

## 命令行
`ros2 topic pub --once  /robot_effort_controller/reference std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: "velocities", size: 4, stride: 1}], data_offset: 0}, data: [0, 0, 0, 0]}'`

`ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

# 官方力控制器

JointGroupEffortController -> ForwardCommandController -> ForwardControllersBase -> controller_interface::ControllerInterface