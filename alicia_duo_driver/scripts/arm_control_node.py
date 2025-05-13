#!/usr/bin/env python3
# coding=utf-8

"""
标准机械臂控制节点 (ROS 2)
接收7自由度(6关节+夹爪)的弧度控制命令 (ArmJointState)，
以及单独的夹爪控制命令 (Float32)，
转换为硬件协议格式并通过 /send_serial_data 发送。
"""

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import UInt8MultiArray, Float32, Bool
# 导入自定义消息 ArmJointState
from alicia_duo_driver.msg import ArmJointState

# 常量定义
RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

# 协议常量
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0xFF
CMD_SERVO_CONTROL = 0x04
# CMD_EXTENDED_CONTROL = 0x06 # Not used in the provided logic
CMD_GRIPPER_CONTROL = 0x02
CMD_ZERO_CAL = 0x03
CMD_DEMO_CONTROL = 0x13 # Corresponds to move_free_callback

class ArmControlNode(Node):
    def __init__(self):
        """初始化节点"""
        super().__init__('arm_control_node') # ROS 2 节点初始化

        # 声明并获取参数
        self.declare_parameter('servo_count', 9)
        self.declare_parameter('debug_mode', False)
        self.servo_count = self.get_parameter('servo_count').get_parameter_value().integer_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.joint_count = 6 # Number of arm joints (excluding gripper)

        self.get_logger().info(f"ArmControlNode Debug mode: {'enabled' if self.debug_mode else 'disabled'}")
        self.get_logger().info(f"Expecting {self.servo_count} servos for frame generation.")

        # 设置数据结构
        self._setup_data()

        # 设置ROS通信
        self._setup_communication()

        self.get_logger().info("Arm Control Node initialized (subscribes to radians, publishes hardware bytes).")

    def _setup_data(self):
        """初始化数据帧模板"""
        # 计算标准舵机控制帧大小
        # Header(1) + CMD(1) + LEN(1) + DATA(servo_count*2) + CHK(1) + FOOTER(1)
        self.frame_size = self.servo_count * 2 + 5
        self.servo_angle_frame = bytearray(self.frame_size) # Use bytearray for mutable bytes
        self.servo_angle_frame[0] = FRAME_HEADER
        self.servo_angle_frame[1] = CMD_SERVO_CONTROL
        self.servo_angle_frame[2] = self.servo_count * 2 # Length field = data bytes
        self.servo_angle_frame[-1] = FRAME_FOOTER

        # 夹爪控制帧 (固定长度 8 bytes: AA 02 03 01 LL HH CHK FF)
        self.gripper_frame = bytearray(8)
        self.gripper_frame[0] = FRAME_HEADER
        self.gripper_frame[1] = CMD_GRIPPER_CONTROL
        self.gripper_frame[2] = 3 # Data length field value (ID + LowByte + HighByte)
        self.gripper_frame[3] = 1 # Gripper ID (assuming always 1 from ROS 1 code)
        self.gripper_frame[-1] = FRAME_FOOTER

        # 关节到舵机的映射表 (从 ROS 1 代码复制)
        # 格式: [(joint_index, direction_multiplier), ...] for each servo
        self.joint_to_servo_map = [
            (0, 1.0),    # Joint 0 -> Servo 0 (positive)
            (0, 1.0),    # Joint 0 -> Servo 1 (positive duplicate)
            (1, 1.0),    # Joint 1 -> Servo 2 (positive)
            (1, -1.0),   # Joint 1 -> Servo 3 (negative)
            (2, 1.0),    # Joint 2 -> Servo 4 (positive)
            (2, -1.0),   # Joint 2 -> Servo 5 (negative)
            (3, 1.0),    # Joint 3 -> Servo 6 (positive)
            (4, 1.0),    # Joint 4 -> Servo 7 (positive)
            (5, 1.0),    # Joint 5 -> Servo 8 (positive)
            # Ensure this list has self.servo_count entries if needed
        ]
        if len(self.joint_to_servo_map) != self.servo_count:
             self.get_logger().warn(f"Mismatch between servo_count ({self.servo_count}) and joint_to_servo_map length ({len(self.joint_to_servo_map)}). Frame generation might be incorrect.")


    def _setup_communication(self):
        """设置 ROS 2 订阅者和发布者"""
        qos_profile = 10 # Basic QoS

        # 发布者 - 发送原始硬件协议字节到串口驱动
        self.serial_pub = self.create_publisher(
            UInt8MultiArray,
            '/send_serial_data', # Topic the C++ driver listens to
            qos_profile)

        # 订阅者 - 接收包含所有关节的标准命令消息
        self.joint_sub = self.create_subscription(
            ArmJointState,
            '/arm_joint_command', # Topic for combined arm commands
            self.joint_command_callback,
            qos_profile)



        # 订阅者 - 零位校准触发器
        self.zero_calibration_sub = self.create_subscription(
            Bool,
            '/zero_calibrate',
            self.zero_calib_callback,
            qos_profile)

        # 订阅者 - 拖动示教模式切换
        self.demo_mode_sub = self.create_subscription(
            Bool,
            '/demonstration', # Topic to enable/disable demo mode
            self.move_free_callback,
            qos_profile)


        self.get_logger().info(f"Publishing hardware commands to {self.serial_pub.topic_name}")

    def calculate_checksum(self, frame_bytearray):
        """计算校验和 (与 ROS 1 校验和函数 *不同*，但与 ROS 1 控制节点使用的 *相同*)"""
        # Checksum is sum of bytes from index 3 up to (but not including) checksum byte at index -2
        # Result is modulo 2 (parity)
        # Ensure input is indexable (like list or bytearray)
        try:
            if len(frame_bytearray) < 5: # Need at least H CMD LEN D CHK F
                 self.get_logger().warn(f"Cannot calculate checksum on short frame (len={len(frame_bytearray)})")
                 return 0
            # Sum bytes from index 3 up to index size-3 (inclusive)
            checksum_sum = sum(frame_bytearray[3:-2])
            return checksum_sum % 2
        except Exception as e:
             self.get_logger().error(f"Error calculating checksum: {e}")
             return 0

    def rad_to_hardware_value(self, angle_rad):
        """将关节弧度 (-pi to pi) 转换为舵机硬件值 (0-4095)"""
        # Convert radians to degrees (-180 to 180)
        angle_deg = angle_rad * RAD_TO_DEG

        # Clamp to valid range (-180 to 180)
        if angle_deg < -180.0 or angle_deg > 180.0:
            self.get_logger().warn(f"Joint angle out of range: {angle_deg:.2f} deg. Clamping to [-180, 180].")
            angle_deg = max(-180.0, min(180.0, angle_deg))

        # Linear mapping from [-180, 180] degrees to [0, 4095] hardware value
        # ROS 1 logic: (angle_deg + 180.0) / 360.0 * 4096
        # 0 deg -> (180/360)*4096 = 2048. Corrected logic should map -180->0, 180->4095
        # hardware_value = int(((angle_deg + 180.0) / 360.0) * 4095) # Map to 0-4095 range
        # Let's precisely use the ROS 1 formula provided:
        value = int((angle_deg + 180.0) / 360.0 * 4096) # Result can be 0-4096

        # Clamp to final 0-4095 range
        return max(0, min(4095, value))

    def rad_to_hardware_value_grip(self, angle_rad):
        """将夹爪弧度 (0 to ~1.74 rad) 转换为硬件值 (2048-2900?)"""
        # Convert radians to degrees (expecting 0 to 100 deg from ROS1 logic)
        angle_deg = angle_rad * RAD_TO_DEG

        # Clamp input degrees based on ROS1 logic (0 to 100 deg)
        # Check if this range is correct for your gripper!
        expected_min_deg = 0.0
        expected_max_deg = 100.0
        if angle_deg < expected_min_deg or angle_deg > expected_max_deg:
            self.get_logger().warn(f"Gripper angle out of expected range [0, 100] deg: {angle_deg:.2f} deg. Clamping.")
            angle_deg = max(expected_min_deg, min(expected_max_deg, angle_deg))

        # Map [0, 100] deg to hardware range [2048, 2900] linearly
        # ROS 1 logic implies: value = angle_deg * 8.52 + 2048
        # Check if 8.52 = (2900-2048)/100 is correct
        hardware_value = int(angle_deg * 8.52 + 2048) # Use the formula derived from ROS 1 gripper callback

        # Clamp to the specific hardware range [2048, 2900]
        # Note: ROS 1 rad_to_hardware_value_grip had a different formula, using the callback logic seems more direct
        return max(2048, min(2900, hardware_value))


    def joint_command_callback(self, msg: ArmJointState):
        """处理 /arm_joint_command 消息"""
        try:
            if self.debug_mode:
                self.get_logger().debug(
                    f"Rcvd ArmJointState: J1={msg.joint1:.3f} J2={msg.joint2:.3f} J3={msg.joint3:.3f} "
                    f"J4={msg.joint4:.3f} J5={msg.joint5:.3f} J6={msg.joint6:.3f} Grip={msg.gripper:.3f}"
                )

            # Extract joint angles into a list
            # Apply inversion based on ROS 1 logic (joint1 negative)
            joint_angles_rad = [
                msg.joint1, msg.joint2, msg.joint3,
                msg.joint4, msg.joint5, msg.joint6
            ]

            # --- Prepare Servo Control Frame (0x04) ---
            # Map each standard joint angle back to the corresponding servos
            for servo_idx in range(self.servo_count):
                if servo_idx < len(self.joint_to_servo_map):
                    mapping = self.joint_to_servo_map[servo_idx]
                    joint_idx, direction = mapping
                    if joint_idx < len(joint_angles_rad):
                        # Apply direction multiplier
                        target_servo_rad = joint_angles_rad[joint_idx] * direction
                        # Convert to hardware value
                        hardware_value = self.rad_to_hardware_value(target_servo_rad)
                    else:
                        self.get_logger().warn(f"Joint index {joint_idx} out of bounds for mapping servo {servo_idx}")
                        hardware_value = 2048 # Default to center?
                else:
                     self.get_logger().warn(f"Servo index {servo_idx} out of bounds for joint_to_servo_map")
                     hardware_value = 2048 # Default to center?

                # Pack hardware value into the frame (Little Endian)
                frame_idx = 3 + servo_idx * 2
                if frame_idx + 1 < len(self.servo_angle_frame):
                    self.servo_angle_frame[frame_idx] = hardware_value & 0xFF      # Low byte
                    self.servo_angle_frame[frame_idx + 1] = (hardware_value >> 8) & 0xFF # High byte
                else:
                     self.get_logger().error(f"Index out of bounds when writing servo {servo_idx} to frame")

            # Calculate and set checksum for the servo frame
            checksum_servo = self.calculate_checksum(self.servo_angle_frame)
            self.servo_angle_frame[-2] = checksum_servo

            # --- Prepare Gripper Control Frame (0x02) ---
            # Convert gripper angle from message to hardware value
            gripper_hw_value = self.rad_to_hardware_value_grip(msg.gripper)
            self.gripper_frame[4] = gripper_hw_value & 0xFF      # Low byte
            self.gripper_frame[5] = (gripper_hw_value >> 8) & 0xFF # High byte

            # Calculate and set checksum for the gripper frame
            checksum_gripper = self.calculate_checksum(self.gripper_frame)
            self.gripper_frame[-2] = checksum_gripper


            # --- Publish Frames ---
            servo_msg_out = UInt8MultiArray()
            servo_msg_out.data = bytes(self.servo_angle_frame) # Convert bytearray to bytes for msg
            self.serial_pub.publish(servo_msg_out)

            gripper_msg_out = UInt8MultiArray()
            gripper_msg_out.data = bytes(self.gripper_frame) # Convert bytearray to bytes for msg
            self.serial_pub.publish(gripper_msg_out)


            if self.debug_mode:
                self._print_debug_info()

        except Exception as e:
            self.get_logger().error(f"Processing joint command exception: {e}")

    def _print_debug_info(self):
        """打印调试信息"""
        servo_hex = " ".join([f"{b:02X}" for b in self.servo_angle_frame])
        self.get_logger().debug(f"Sent Servo Frame (0x04): {servo_hex}")
        gripper_hex = " ".join([f"{b:02X}" for b in self.gripper_frame])
        self.get_logger().debug(f"Sent Gripper Frame (0x02): {gripper_hex}")

    def frame_ge(self, control_cmd, control_data=0x00, check=False):
        """Helper to generate simple command frames (like ROS 1)"""
        # Frame: AA CMD LEN DATA CHK FF (Length 6)
        frame_d = bytearray(6)
        frame_d[0] = FRAME_HEADER
        frame_d[1] = control_cmd
        frame_d[2] = 0x01  # Data length is 1 byte
        frame_d[3] = control_data & 0xFF # Ensure data is single byte

        if check:
            # Calculate checksum based on this specific frame structure
            # Sum is only over DATA byte at index 3
            checksum_sum = frame_d[3]
            frame_d[-2] = checksum_sum % 2
        else:
            frame_d[-2] = 0x00 # Checksum byte is 0 if check=False

        frame_d[-1] = FRAME_FOOTER

        frame_d_msg = UInt8MultiArray()
        frame_d_msg.data = bytes(frame_d) # Convert to bytes for message
        return frame_d_msg

    def zero_calib_callback(self, msg: Bool):
        """处理 /zero_calibrate 消息"""
        try:
            if msg.data:
                self.get_logger().info("Received Zero Calibration command.")
                zero_calib_msg = self.frame_ge(CMD_ZERO_CAL) # Generate 0x03 command frame
                self.serial_pub.publish(zero_calib_msg)
                if self.debug_mode:
                     self.get_logger().debug(f"Sent Zero Calib Frame (0x03): {' '.join([f'{b:02X}' for b in zero_calib_msg.data])}")
            # else: Do nothing if msg.data is false?

        except Exception as e:
            self.get_logger().error(f"Processing zero calibration command exception: {e}")

    def move_free_callback(self, msg: Bool):
        """处理 /demonstration 消息 (切换拖动示教模式)"""
        try:
            if msg.data:
                self.get_logger().info("Enabling Demonstration Mode (Zero Torque).")
                # Data = 0x00 for zero torque
                move_msg = self.frame_ge(CMD_DEMO_CONTROL, control_data=0x00, check=False) # Check=False based on ROS 1
            else:
                self.get_logger().info("Disabling Demonstration Mode (Full Torque).")
                # Data = 0x01 for full torque
                move_msg = self.frame_ge(CMD_DEMO_CONTROL, control_data=0x01, check=True) # Check=True based on ROS 1

            self.serial_pub.publish(move_msg)
            if self.debug_mode:
                 self.get_logger().debug(f"Sent Demo Mode Frame (0x13): {' '.join([f'{b:02X}' for b in move_msg.data])}")

        except Exception as e:
            self.get_logger().error(f"Processing demonstration mode command exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmControlNode()
        node.get_logger().info("Arm Control Node started.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception: {e}")
        else: print(f"Exception during node initialization: {e}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()