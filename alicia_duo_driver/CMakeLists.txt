cmake_minimum_required(VERSION 3.8)
project(alicia_duo_driver)

# 查找ament和依赖的ROS包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# --- 使用 PkgConfig 查找 libserial ---
# 1. 查找 pkg-config 工具
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# 2. 使用 pkg-config 查找 libserial 库 (对应 libserial.pc 文件)
#    这会定义 PC_SERIAL_FOUND, PC_SERIAL_INCLUDE_DIRS, PC_SERIAL_LIBRARY_DIRS, PC_SERIAL_LIBRARIES 等变量
pkg_check_modules(PC_SERIAL REQUIRED IMPORTED_TARGET libserial)
# IMPORTED_TARGET 会创建一个名为 PkgConfig::PC_SERIAL 的目标，便于链接

# 添加包含目录
include_directories(include)
# 添加 libserial 的 include 目录 (由 pkg_check_modules 设置)
include_directories(${PC_SERIAL_INCLUDE_DIRS})

# 添加可执行文件
add_executable(${PROJECT_NAME}_node src/serial_server_node.cpp src/serial_server_main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

# 链接 Serial 库
# 使用 pkg-config 找到的导入目标 PkgConfig::PC_SERIAL

target_link_libraries(${PROJECT_NAME}_node ${PC_SERIAL_LIBRARIES}) # Use plain signature with variable
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  # Add other .msg/.srv/.action files here if needed
  DEPENDENCIES std_msgs # Add if your msg uses other msgs like std_msgs/Header
)
# 安装目标文件
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts
# Install Python scripts
install(PROGRAMS
  scripts/serial_dispatcher_node.py
  scripts/joint_state_publisher_node.py
  scripts/arm_control_node.py # <--- 添加控制节点脚本
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



# Finalize ament packaging
ament_export_dependencies(std_msgs)
ament_package()