cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  nav_msgs
  tf2
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
# 测试部分
# find_package(ament_cmake_gmock REQUIRED)
# find_package(ros2_control_test_assets REQUIRED)

# find_package(rsl REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add my_controller library related compile commands
generate_parameter_library(my_controller_parameters
  src/my_controller.yaml
  include/robot_controller/validate_my_controller_parameters.hpp
)
add_library(
  my_controller
  SHARED
  src/my_controller.cpp
  src/odometry.cpp
)
target_include_directories(my_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(my_controller my_controller_parameters)
ament_target_dependencies(my_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(my_controller PRIVATE "MY_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface robot_controller.xml)

install(
  TARGETS
  my_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)

#   ament_add_gmock(test_load_my_controller test/test_load_my_controller.cpp)
#   target_include_directories(test_load_my_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_my_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   add_rostest_with_parameters_gmock(test_my_controller test/test_my_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/my_controller_params.yaml)
#   target_include_directories(test_my_controller PRIVATE include)
#   target_link_libraries(test_my_controller my_controller)
#   ament_target_dependencies(
#     test_my_controller
#     controller_interface
#     hardware_interface
#   )

#   add_rostest_with_parameters_gmock(test_my_controller_preceeding test/test_my_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/my_controller_preceeding_params.yaml)
#   target_include_directories(test_my_controller_preceeding PRIVATE include)
#   target_link_libraries(test_my_controller_preceeding my_controller)
#   ament_target_dependencies(
#     test_my_controller_preceeding
#     controller_interface
#     hardware_interface
#   )
# endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  my_controller
)

ament_package()
