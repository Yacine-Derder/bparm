cmake_minimum_required(VERSION 3.8)
project(compute_traj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Find dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Generate message code ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ComputeTrajRequest.msg"
  DEPENDENCIES geometry_msgs
)

# === Declare C++ executable ===
add_executable(compute_traj_node src/compute_traj.cpp)
ament_target_dependencies(compute_traj_node 
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  trajectory_msgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
)

# === Link generated message types ===
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(compute_traj_node "${cpp_typesupport_target}")

# === Install the executable ===
install(TARGETS compute_traj_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Linting/test support ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
