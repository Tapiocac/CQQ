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


clone ros_team_workspace

安装gmock
`sudo apt install libgmock-dev`


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


input_ref_通过订阅话题接收上位机的命令，ros2_control根据yaml文件中的配置声明来生成接收和下发到硬件的接口





# 官方力控制器

JointGroupEffortController -> ForwardCommandController -> ForwardControllersBase -> controller_interface::ControllerInterface