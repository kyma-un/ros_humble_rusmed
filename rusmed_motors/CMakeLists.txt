cmake_minimum_required(VERSION 3.8)
project(rusmed_motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(ros2_control_cmake REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(controller_manager )

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## Compile ---
add_library(
  axial_joint_hw_interface
  SHARED
  hardware/axial_joint_hw_interface.cpp
)


target_compile_features(axial_joint_hw_interface PUBLIC cxx_std_17)
target_include_directories(axial_joint_hw_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/axial_joint_hw_interface>
)

ament_target_dependencies(axial_joint_hw_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
    
)

target_link_libraries(
  axial_joint_hw_interface
  pigpiod_if2
)



# target_link_libraries(axial_joint_hw_interface PUBLIC
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
# )



# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rusmed_motors.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/axial_joint_hw_interface
)


install(DIRECTORY
  launch
  config
  hardware
  description
  DESTINATION share/${PROJECT_NAME}/
)

# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/axial_joint_hw_interface
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/axial_joint_hw_interface)
install(TARGETS axial_joint_hw_interface
  EXPORT export_axial_joint_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



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

add_executable(stepper src/stepper.cpp)
# target_link_libraries(stepper i2c)
ament_target_dependencies(stepper rclcpp std_msgs)


target_link_libraries(stepper /usr/lib/aarch64-linux-gnu/libi2c.so)

install(TARGETS
  stepper
  DESTINATION lib/${PROJECT_NAME})

## EXPORTS
ament_export_targets(export_axial_joint_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
