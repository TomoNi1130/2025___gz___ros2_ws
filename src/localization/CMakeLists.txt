cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interface_pkg REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(sub
  src/localization.cpp
)
ament_target_dependencies(sub
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  interface_pkg
  Eigen3
)

add_executable(main
  src/new_localization.cpp
)
ament_target_dependencies(main
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  interface_pkg
  Eigen3
)

install(TARGETS
main
sub
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
