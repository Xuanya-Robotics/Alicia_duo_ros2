cmake_minimum_required(VERSION 3.8)
project(alicia_duo_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(alicia_duo_driver REQUIRED)

# Create the hardware interface library
add_library(${PROJECT_NAME} SHARED
  src/alicia_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  alicia_duo_driver
)

pluginlib_export_plugin_description_file(hardware_interface alicia_hardware_interface.xml)

# Install the library
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# BUILD TEST ALWAYS (outside BUILD_TESTING block)
add_executable(test_alicia_hardware_interface
  test/test_alicia_hardware_interface.cpp
)

# NOW configure the test executable AFTER creating it
target_include_directories(test_alicia_hardware_interface PRIVATE
  include
)

ament_target_dependencies(test_alicia_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  alicia_duo_driver
)

target_link_libraries(test_alicia_hardware_interface
  ${PROJECT_NAME}
)

# Install the test executable
install(TARGETS test_alicia_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Remove this line: find_package(ament_cmake_gmock REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()