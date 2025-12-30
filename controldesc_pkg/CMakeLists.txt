cmake_minimum_required(VERSION 3.8)
project(controldesc_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---------- C++ ----------
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp geometry_msgs)

install(
  TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Python ----------
install(
  PROGRAMS
  src/robot_controller.py
  src/A_star_controller/A_star_mejor.py
  src/A_star_controller/set_initial_pose.py
  src/A_star_controller/map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
# ---------- Maps ----------
install(
  DIRECTORY src/A_star_controller/maps
  DESTINATION share/${PROJECT_NAME}
)
# ---------- Launch files ----------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# ---------- Testing ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

