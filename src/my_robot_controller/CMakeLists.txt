cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installer les dossiers
install(
  DIRECTORY description launch config rviz web_interface worlds scripts  nav2_web_interface
  DESTINATION share/${PROJECT_NAME}
)

# Installer le script Python
install(PROGRAMS
  web_interface/web_control_interface.py
  nav2_web_interface/unified_web_interface.py
  nav2_web_interface/nav2_hospital_interface.py
  scripts/save_map.sh
  scripts/camera_viewer.py
 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()