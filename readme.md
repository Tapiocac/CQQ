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