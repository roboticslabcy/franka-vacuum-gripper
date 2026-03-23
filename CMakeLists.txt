cmake_minimum_required(VERSION 3.5)
project(franka_vacuum_gripper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Franka 0.19.0 REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VacuumGripperState.msg"
  "srv/Vacuum.srv"
  "srv/DropOff.srv"
  DEPENDENCIES std_msgs
)

# Node library (composable)
add_library(vacuum_gripper_node SHARED
  src/vacuum_gripper_node.cpp)

target_include_directories(vacuum_gripper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(vacuum_gripper_node Franka::Franka)

ament_target_dependencies(vacuum_gripper_node
  rclcpp
  rclcpp_components
  std_srvs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vacuum_gripper_node ${cpp_typesupport_target})

rclcpp_components_register_node(
  vacuum_gripper_node
  PLUGIN "franka_vacuum_gripper::VacuumGripperNode"
  EXECUTABLE franka_vacuum_gripper_node)

install(TARGETS vacuum_gripper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/vacuum_gripper_demo.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
