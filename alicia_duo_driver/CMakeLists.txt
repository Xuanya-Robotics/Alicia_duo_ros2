cmake_minimum_required(VERSION 3.8)
project(alicia_duo_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all necessary packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_SERIAL REQUIRED IMPORTED_TARGET libserial)

# 2. Generate C++ headers from the .msg file
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  DEPENDENCIES std_msgs
)

# 3. Build the reusable serial communicator library
add_library(serial_communicator_lib SHARED src/serial_communicator.cpp)
target_include_directories(serial_communicator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PC_SERIAL_INCLUDE_DIRS}
)
# --- CORRECTED LINKING ---
# Combine all dependencies into a single call with keywords
target_link_libraries(serial_communicator_lib PUBLIC
  rclcpp::rclcpp
  PkgConfig::PC_SERIAL
)

# 4. Build the main driver node as an EXECUTABLE
add_executable(alicia_duo_driver_node src/alicia_duo_driver_node.cpp)
ament_target_dependencies(alicia_duo_driver_node
  rclcpp
  std_msgs
)
target_link_libraries(alicia_duo_driver_node
  serial_communicator_lib
)
# --- REPLACED DEPRECATED COMMAND ---
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(alicia_duo_driver_node ${cpp_typesupport_target})

# 5. Build the ros2_control hardware interface as a SHARED LIBRARY
add_library(alicia_hardware_interface SHARED src/alicia_hardware_interface.cpp)
ament_target_dependencies(alicia_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(alicia_hardware_interface
  serial_communicator_lib
)
# --- REPLACED DEPRECATED COMMAND ---
target_link_libraries(alicia_hardware_interface ${cpp_typesupport_target})

# 6. Set up installation rules
install(TARGETS
  alicia_duo_driver_node
  alicia_hardware_interface
  serial_communicator_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(hardware_interface alicia_hardware_interface.xml)

# 7. Finalize the package
ament_package()