cmake_minimum_required(VERSION 3.8)
project(controldesc_pkg)

find_package(ament_cmake REQUIRED)

# ---------- Python ----------
install(
  PROGRAMS
  src/robot_controller.py
  src/evaluation.py
  src/A_star_controller/comparaciones.py
  src/A_star_controller/A_star_mejor.py
  src/A_star_controller/Theta_star.py
  src/A_star_controller/Theta_star2.py
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
  # Eliminamos cpplint porque no hay C++
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
