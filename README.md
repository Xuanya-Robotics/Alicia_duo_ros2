
# 机械臂控制系统 v1.2 (ROS 2 Humble 移植版)

## 1. 产品概述

机械臂控制系统 v1.2 是一套基于 ROS 2 Humble 的完整解决方案，专为6自由度机械臂及夹爪设计。系统提供标准化的 ROS 2 接口，使用弧度作为统一角度单位，简化了机械臂的控制过程，屏蔽底层通信细节，让用户专注于应用开发。

**核心功能：**

*   6自由度机械臂实时控制
*   全关节状态实时反馈 (发布 `/arm_joint_state`)
*   标准 ROS 2 接口
*   串口自动重连机制
*   通过 Launch 文件统一启动和配置

## 2. 系统架构

系统由四个主要节点组成，它们都在 `alicia_duo_driver` 包中，共同构成完整的控制链路：

| 节点名称 (Launch 中) | 可执行文件                     | 语言 | 功能描述                                                                 |
| :------------------- | :------------------------------- | :--- | :----------------------------------------------------------------------- |
| `serial_driver`      | `alicia_duo_driver_node`       | C++  | 负责与硬件串口通信，处理原始数据帧的收发（订阅 `/send_serial_data`，发布 `/read_serial_data`）。 |
| `serial_dispatcher`  | `serial_dispatcher_node.py`    | Py   | 订阅 `/read_serial_data`，根据命令码将原始数据帧分类转发至对应处理节点的话题（如 `/servo_states`, `/gripper_angle`, `/error_frame_deal`）。 |
| `joint_state_publisher` | `joint_state_publisher_node.py`| Py   | 订阅 `/servo_states` 和 `/gripper_angle`，处理硬件状态数据，转换为标准关节状态（弧度）并发布到 `/arm_joint_state`。 |
| `arm_control`        | `arm_control_node.py`          | Py   | 订阅标准关节命令 (`/arm_joint_command`)、单独夹爪命令 (`/gripper_control`) 及其他控制命令，转换为硬件协议格式并发布到 `/send_serial_data`。 |



## 3. 接口规范

### 3.1 主要用户接口

用户主要通过以下话题与系统交互：

| 话题名称             | 消息类型                           | 方向        | 描述                                                                |
| :------------------- | :--------------------------------- | :---------- | :------------------------------------------------------------------ |
| `/arm_joint_state`   | `alicia_duo_driver/ArmJointState`  | **读取**    | 订阅此话题以获取机械臂所有关节和夹爪的**当前状态**（弧度单位）。    |
| `/arm_joint_command` | `alicia_duo_driver/ArmJointState`  | **发送**    | 发布到此话题以**同时控制**机械臂的6个主要关节和夹爪的目标位置（弧度）。 |
| `/zero_calibrate`    | `std_msgs/msg/Bool`                | **发送**    | 发布 `true` 到此话题以触发机械臂的零位校准命令。                   |
| `/demonstration`     | `std_msgs/msg/Bool`                | **发送**    | 发布 `true` 使能，发布 `false`失能。 |


### 3.2 内部接口 (调试用)

| 话题名称             | 消息类型                           | 方向                      | 描述                                           |
| :------------------- | :--------------------------------- | :------------------------ | :--------------------------------------------- |
| `/send_serial_data`  | `std_msgs/msg/UInt8MultiArray`     | `arm_control` -> `serial_driver` | 控制节点生成的发送给硬件的原始字节帧。         |
| `/read_serial_data`  | `std_msgs/msg/UInt8MultiArray`     | `serial_driver` -> `serial_dispatcher` | C++驱动从硬件读取并验证通过的原始字节帧。      |
| `/servo_states`      | `std_msgs/msg/UInt8MultiArray`     | `serial_dispatcher` -> `joint_state_publisher` | 分发出的原始舵机状态帧 (CMD 0x04)。            |                |
| `/servo_states_main` | `std_msgs/msg/Float32MultiArray`   | `joint_state_publisher` -> | 兼容 ROS 1 的关节状态数组发布（弧度）。      |

### 3.3 消息类型定义 (`alicia_duo_driver/msg/ArmJointState.msg`)

```protobuf
# 标准机械臂关节状态/命令消息 (ROS 2)
# 所有角度使用弧度作为单位

std_msgs/Header header

# 六个主要关节角度 (弧度)
float32 joint1  # 底座旋转关节
float32 joint2  # 肩部关节
float32 joint3  # 肘部关节
float32 joint4  # 腕部旋转关节
float32 joint5  # 腕部俯仰关节
float32 joint6  # 腕部翻转关节

# Button states (从夹爪状态帧中获取)
int32 but1
int32 but2

# 夹爪角度 (弧度)
float32 gripper

# 可选的运动控制参数 (当前版本代码中未使用 time 字段)
# float32 time    # 运动时间(秒)，默认为0表示立即执行
```

### 3.4 数据单位与限制

所有用户接口的角度值使用**弧度**作为标准单位：

| 参数名称        | 角度范围 (弧度) | 角度范围 (度)   | 备注                               |
| :-------------- | :-------------- | :-------------- | :--------------------------------- |
| `joint1`~`joint6` | -π 到 +π       | -180° 到 +180° | 控制时超出范围会被自动截断         |
| `gripper`       | 0 到 ~1.745    | 0° 到 100°      | 控制时超出范围会被自动截断（硬件限制） |

**注意：** 夹爪的角度范围受限于 0-100 度。

## 4. 系统部署与安装

### 4.1 系统依赖安装

*   **ROS 2 Humble:** 确保已安装 ROS 2 Humble Desktop Full。
*   **系统库:** 安装 `libserial-dev` 用于 C++ 串口通信。
    ```bash
    sudo apt update
    sudo apt install libserial-dev
    ```
*   **Python 库:** 安装 `numpy`。
    ```bash
    sudo apt install python3-numpy # 或者 pip3 install numpy
    ```
    (`rclpy` 和 `std_msgs` Python 库通常随 ROS 2 Desktop 一起安装)。

### 4.2 获取源代码

1.  **克隆源代码:** 
    ```bash
    git clone https://github.com/Xianova-Robotics/Alicia_duo_ros2.git
    ```

### 4.3 编译工作空间

```bash
cd ~/Alicia_duo_leader_ros2
colcon build --packages-select alicia_duo_driver --symlink-install
```
*   `--symlink-install` 允许你修改 Python 或 Launch 文件后通常无需重新编译（但修改 C++/CMakeLists.txt/msg 后必须重新编译）。

### 4.4 配置环境变量

将工作空间的 `setup` 文件添加到你的 shell 配置文件中，以便每次打开新终端时自动加载：

```bash
echo "source ~/Alicia_duo_leader_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
# 如果你使用 zsh，请使用 setup.zsh 和 .zshrc
```

### 4.5 设置串口权限

为了让 ROS 节点能够访问串口设备 (如 `/dev/ttyUSB0`)，需要设置权限：

*   **方法1：添加用户到 `dialout` 组 (推荐，永久有效):**
    ```bash
    sudo usermod -a -G dialout $USER
    ```
    **重要:** 执行此命令后，你需要**完全注销当前用户并重新登录**才能使组更改生效。

*   **方法2：创建 udev 规则 (推荐，永久有效):**
    创建一个文件 `/etc/udev/rules.d/99-serial.rules` 并添加以下内容（如果文件已存在，请编辑）：
    ```
    KERNEL=="ttyUSB*", MODE="0666"
    KERNEL=="ttyACM*", MODE="0666"
    ```
    然后重新加载规则：
    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    这会在设备插入时自动设置权限。

*   **方法3：临时设置权限 (每次重启或重插后都需要):**
    ```bash
    sudo chmod 666 /dev/ttyUSB0 # 将 ttyUSB0 替换为你的设备
    ```

### 4.6 脚本执行权限

确保所有 Python 脚本都具有执行权限（`colcon build` 和 `install(PROGRAMS ...)` 通常会处理，但手动检查更保险）：

```bash
chmod +x ~/Alicia_duo_leader_ros2/src/alicia_duo_driver/scripts/*.py
```

### 4.7 硬件连接验证

1.  将机械臂通过 USB 连接至计算机。
2.  检查串口是否被系统识别：
    ```bash
    ls -l /dev/ttyUSB* /dev/ttyACM*
    ```
    记下你的设备名称 (例如 `ttyUSB0`)。

## 5. 系统启动

使用提供的 Launch 文件启动所有节点：

```bash
ros2 launch alicia_duo_driver serial_driver_launch.py [参数名称:=参数值 ...]
```

**可配置参数 (Launch 参数):**

| 参数名称         | 默认值     | 说明                                                          |
| :--------------- | :--------- | :------------------------------------------------------------ |
| `port`           | `ttyUSB0`  | 串口设备名称 (位于 `/dev/` 下)                                |
| `baudrate`       | `921600`   | 串口波特率                                                     |
| `timeout_ms`     | `1000`     | C++ 驱动读取串口的超时时间 (毫秒)                            |
| `py_debug`       | `false`    | 是否为所有 Python 节点启用详细的调试日志 (`true` 或 `false`)  |
| `servo_count`    | `9`        | `joint_state_publisher` 和 `arm_control` 期望的舵机数量         |
| `rate_limit_sec` | `0.01`     | `joint_state_publisher` 处理数据的最小时间间隔 (秒)            |

**启动示例:**

*   使用默认设置：
    ```bash
    ros2 launch alicia_duo_driver serial_driver_launch.py
    ```
*   指定端口并启用 Python 调试：
    ```bash
    ros2 launch alicia_duo_driver serial_driver_launch.py port:=ttyACM0 py_debug:=true
    ```
*   修改波特率：
    ```bash
    ros2 launch alicia_duo_driver serial_driver_launch.py baudrate:=115200
    ```

## 6. 使用方法

### 6.1 发送控制命令 (Python 示例)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from alicia_duo_driver.msg import ArmJointState
from std_msgs.msg import Float32

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander_example')
        # Publisher for combined joint commands
        self.cmd_pub = self.create_publisher(ArmJointState, '/arm_joint_command', 10)
        # Publisher for separate gripper commands
        self.gripper_pub = self.create_publisher(Float32, '/gripper_control', 10)
        self.get_logger().info('Arm Commander Node Started. Waiting 2s before sending commands...')
        # Allow time for other nodes to start
        self.create_timer(2.0, self.send_commands_callback)

    def send_commands_callback(self):
        self.get_logger().info('Sending commands...')

        # --- Example 1: Send combined joint command (all joints + gripper) ---
        cmd = ArmJointState()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Set joint angles (convert degrees to radians)
        cmd.joint1 = math.radians(0.0)    # Base
        cmd.joint2 = math.radians(0.0)    # Shoulder
        cmd.joint3 = math.radians(0.0)    # Elbow
        cmd.joint4 = math.radians(0.0)    # Wrist Rotate
        cmd.joint5 = math.radians(30.0)   # Wrist Pitch
        cmd.joint6 = math.radians(0.0)    # Wrist Roll
        cmd.gripper = math.radians(10.0)  # Gripper 10 degrees open (approx 0.17 rad)

        self.cmd_pub.publish(cmd)
        self.get_logger().info('Published command to /arm_joint_command')
        time.sleep(3.0) # Wait a bit

        # --- Example 2: Send separate gripper command ---
        gripper_cmd = Float32()
        gripper_cmd.data = math.radians(80.0) # Gripper 80 degrees open (approx 1.4 rad)

        self.gripper_pub.publish(gripper_cmd)
        self.get_logger().info(f'Published command {gripper_cmd.data:.3f} rad to /gripper_control')
        time.sleep(3.0)

        # --- Example 3: Send another combined command (gripper value ignored if using separate control often) ---
        cmd.joint5 = math.radians(-30.0)
        cmd.gripper = math.radians(0.0) # Set gripper to 0 (closed) via combined message
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Published another command to /arm_joint_command')


        # Shutdown after sending commands
        self.get_logger().info('Commands sent. Shutting down commander.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ensure shutdown happens even if spin wasn't reached
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.2 读取关节状态 (Python 示例)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from alicia_duo_driver.msg import ArmJointState

class ArmStateMonitor(Node):
    def __init__(self):
        super().__init__('arm_state_monitor')
        self.subscription = self.create_subscription(
            ArmJointState,
            '/arm_joint_state',
            self.joint_state_callback,
            10) # QoS depth
        self.get_logger().info('Arm State Monitor Started. Listening to /arm_joint_state...')

    def joint_state_callback(self, msg: ArmJointState):
        # Convert radians to degrees for readability
        j1 = math.degrees(msg.joint1)
        j2 = math.degrees(msg.joint2)
        j3 = math.degrees(msg.joint3)
        j4 = math.degrees(msg.joint4)
        j5 = math.degrees(msg.joint5)
        j6 = math.degrees(msg.joint6)
        grip = math.degrees(msg.gripper)

        log_str = (
            f"Current State (degrees):\n"
            f"  Joints: [{j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f}, {j5:.2f}, {j6:.2f}]\n"
            f"  Gripper: {grip:.2f}\n"
            f"  Buttons: [{msg.but1}, {msg.but2}]"
        )
        self.get_logger().info(log_str)

def main(args=None):
    rclpy.init(args=args)
    node = ArmStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.3 命令行测试方法

*   **发送关节归零命令:**
    ```bash
    ros2 topic pub /arm_joint_command alicia_duo_driver/msg/ArmJointState '{joint1: 0.0, joint2: 0.0, joint3: 0.0, joint4: 0.0, joint5: 0.0, joint6: 0.0, gripper: 0.0}' --once
    ```
*   **触发零位校准:**
    ```bash
    ros2 topic pub /zero_calibrate std_msgs/msg/Bool "{data: true}" --once
    ```
*   **失能:**
    ```bash
    ros2 topic pub /demonstration std_msgs/msg/Bool "{data: true}" --once
    ```
*   **使能:**
    ```bash
    ros2 topic pub /demonstration std_msgs/msg/Bool "{data: false}" --once
    ```
*   **监听关节状态:**
    ```bash
    ros2 topic echo /arm_joint_state
    ```

## 7. 关节映射说明

| 接口名称       | 关节序号  | 物理位置   | 运动范围 (度)   |
| :------------- | :------ | :--------- | :-------------- |
| `joint1`       | 关节 0   | 底座旋转   | -180° 到 +180°  |
| `joint2`       | 关节 1   | 肩部关节   | -180° 到 +180°  |
| `joint3`       | 关节 2   | 肘部关节   | -180° 到 +180°  |
| `joint4`       | 关节 3   | 腕部旋转   | -180° 到 +180°  |
| `joint5`       | 关节 4   | 腕部俯仰   | -180° 到 +180°  |
| `joint6`       | 关节 5   | 腕部翻转   | -180° 到 +180°  |
| `gripper`      | 夹爪     | 末端执行器 | 0° 到 100°      |

## 8. 故障排除

### 8.1 常见问题

| 问题描述         | 可能原因                                         | 解决方法                                                                                             |
| :--------------- | :----------------------------------------------- | :--------------------------------------------------------------------------------------------------- |
| Launch 启动失败  | 脚本未找到、权限不足、依赖缺失                 | 检查 CMakeLists.txt 安装指令、`chmod +x`、`colcon build` 是否成功、`package.xml` 依赖是否完整             |
| 无法连接到串口   | 权限不足、端口名称错误、设备未连接               | 检查串口权限设置 (`dialout` 组或 udev 规则)、确认 `port` 参数、检查物理连接和 `ls /dev/tty*`           |
| 通信中断/不稳定 | USB 连接问题、串口线质量                          | 使用高质量 USB 线缆、尝试不同 USB 端口、检查连接是否松动                                                   |
| 硬件无响应       | 波特率错误、校验和错误、协议不匹配、硬件状态问题 | 确认 `baudrate` 参数、**仔细核对硬件协议文档修正校验和及转换逻辑**、检查硬件供电和状态、使用 `py_debug` 查看发送的帧 |
| 运动范围/方向错误| 角度转换错误、关节映射错误                       | **仔细核对硬件协议文档修正 `rad_to_hardware_value` / `_grip` 函数**、检查 `joint_to_servo_map`             |
| Checksum 失败   | 校验和算法/范围不匹配、通信干扰                 | **首要任务：根据硬件文档修正校验和计算!** (Python 控制节点 + C++ 驱动节点都需要)、检查硬件连接                 |

### 8.2 诊断方法

*   **启用调试模式:** 在 launch 命令后添加 `py_debug:=true` 以查看 Python 节点的详细日志。C++ 节点的 debug 由其内部参数控制（目前也链接到 `py_debug`）。
    ```bash
    ros2 launch alicia_duo_driver serial_driver_launch.py port:=ttyUSB0 py_debug:=true
    ```
*   **检查节点状态:**
    ```bash
    ros2 node list
    ros2 node info /serial_driver  # 或其他节点名
    ```
*   **监控内部话题:**
    ```bash
    ros2 topic echo /read_serial_data  # 查看 C++ 驱动接收并验证通过的帧
    ros2 topic echo /send_serial_data  # 查看控制节点发送的原始帧
    ros2 topic echo /servo_states      # 查看分发器转发的舵机状态帧
    ros2 topic echo /gripper_angle     # 查看分发器转发的夹爪状态数据
    ros2 topic echo /arm_joint_state   # 查看最终发布的关节状态
    ```
*   **使用串口监控工具:** 如 `minicom`, `picocom` (Linux) 或其他工具，直接连接串口，手动发送十六进制命令帧进行测试。

## 9. 注意事项

1.  **安全第一:** 确保机械臂工作区域安全，无障碍物或人员。
2.  **角度限制:** 命令应在有效范围内，尤其注意夹爪的 0-100 度限制。
3.  **数据单位:** 用户接口使用**弧度 (radians)**。
4.  **校验和与转换:** **务必根据你的具体硬件协议文档，仔细验证并修改 Python 控制节点 (`arm_control_node.py`) 和 C++ 驱动节点 (`serial_server_node.cpp`) 中的校验和计算 (`calculate_checksum`/`sumElements`)、角度到硬件值的转换 (`rad_to_hardware_value`/`_grip`) 以及关节到舵机的映射 (`joint_to_servo_map`)。这是确保系统正常工作的关键。**
5.  **启动顺序:** 使用 launch 文件可确保节点按合理顺序（或并行）启动。
6.  **串口权限:** 首次使用或更换 USB 端口可能需要重新配置权限。
7.  **停止:** 使用 `Ctrl+C` 在运行 launch 文件的终端中安全地停止所有节点。

## 10. 附录

### 10.1 系统依赖

*   ROS 2 Humble Hawksbill (Desktop Full 推荐)
*   `libserial-dev` (系统库, 通过 `apt` 安装)
*   `python3-numpy` (Python 库, 通过 `apt` 或 `pip` 安装)

### 10.2 源码与文档

*   源码位于 `alicia_duo_driver` 包内。
*   本文档 (`README.md`)。

### 10.3 术语表

| 术语       | 说明                                                     |
| :--------- | :------------------------------------------------------- |
| 关节 (Joint) | 机械臂的可运动部分 (0-5 对应 joint1-6)                     |
| 夹爪 (Gripper)| 末端执行器                                               |
| 弧度 (Radian)| 角度的标准单位 (π 弧度 = 180 度)                         |
| 话题 (Topic) | ROS 2 中节点间传递消息的通道                             |
| 节点 (Node)  | ROS 2 系统中的一个独立运行的程序                         |
| 发布 (Publish) | 向话题发送消息的操作                                     |
| 订阅 (Subscribe)| 从话题接收消息的操作                                     |
| Launch 文件 | 用于配置和启动一个或多个 ROS 2 节点的脚本 (通常是 Python) |

### 10.4 重要参数参考

| 参数         | 值                | 说明                                                         |
| :----------- | :---------------- | :----------------------------------------------------------- |
| 默认波特率   | 921600            | 可通过 launch 参数 `baudrate` 修改                           |
| 关节角度限制 | -π 到 +π          | 控制节点会截断超出此范围的输入                               |
| 夹爪角度限制 | 0 到 ~1.745 rad  | 约 0° 到 100°，控制节点会截断                                  |
| 舵机值范围   | 0 - 4095          | 对应 -180° 到 +180° (根据 `rad_to_hardware_value` 函数)      |
| 夹爪值范围   | 2048 - 2900       | 对应 0° 到 100° (根据 `rad_to_hardware_value_grip` 函数)     |
| 默认串口     | `ttyUSB0`         | 可通过 launch 参数 `port` 修改                               |
| 通信协议帧   | AA CMD LEN ... CHK FF | 基本帧结构，具体内容和校验和**需参照硬件文档**              |

### 10.5 版本信息

*   文档版本：v1.2 (ROS 2 移植版)
*   软件版本：v1.2
*   适配 ROS 版本：Humble Hawksbill
*   更新日期：2025年5月6日 (根据当前对话日期)
*   更新记录：
    *   v1.2 (ROS 2): 移植 ROS 1 系统到 ROS 2 Humble，整合为单个包，添加 launch 文件。
    *   (继承自 ROS 1 v1.2)

```