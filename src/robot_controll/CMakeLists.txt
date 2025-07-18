cmake_minimum_required(VERSION 3.8)
project(robot_controll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interface REQUIRED)

add_library(wheel_controll SHARED
src/wheels_controll.cpp)

ament_target_dependencies(wheel_controll
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  interface
)

rclcpp_components_register_node(
    wheel_controll
    PLUGIN "wheels_controll::WheelCon"
    EXECUTABLE wheel_controll_node
)

ament_export_targets(export_component)
install(TARGETS wheel_controll
        EXPORT export_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(set_goal SHARED
   src/set_goal.cpp)

ament_target_dependencies(set_goal
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2 tf2_ros
  Eigen3
  visualization_msgs
  interface
)

rclcpp_components_register_node(
    set_goal
    PLUGIN "set_goal::SetGoal"
    EXECUTABLE set_goal_node
)

ament_export_targets(export_component)
install(TARGETS set_goal
        EXPORT export_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

include_directories(${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
