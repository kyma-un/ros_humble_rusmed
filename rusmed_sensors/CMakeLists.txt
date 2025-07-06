cmake_minimum_required(VERSION 3.8)
project(rusmed_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
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

add_executable(imu src/imu.cpp)
ament_target_dependencies(imu rclcpp std_msgs sensor_msgs)
target_link_libraries(imu pigpiod_if2)

add_executable(imu_tf_broadcaster src/imu_tf_broadcaster.cpp)
ament_target_dependencies(imu_tf_broadcaster rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros )
target_link_libraries(imu_tf_broadcaster pigpiod_if2)

install(TARGETS
  imu
  imu_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})



ament_package()
