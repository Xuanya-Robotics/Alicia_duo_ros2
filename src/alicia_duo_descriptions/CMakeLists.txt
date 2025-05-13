cmake_minimum_required(VERSION 3.8)
project(alicia_duo_descriptions)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python launch files
install(PROGRAMS
  launch/display.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()