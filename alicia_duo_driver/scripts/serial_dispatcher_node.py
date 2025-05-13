#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt32MultiArray
import time
import sys # For sys.exit

class SerialDispatcherNode(Node):
    def __init__(self):
        super().__init__('serial_dispatcher_node') # Initialize Node with name

        # Declare and get parameters (ROS 2 style)
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.get_logger().info(f"Dispatcher Debug mode: {'enabled' if self.debug_mode else 'disabled'}") # Added prefix for clarity

        # Statistics - for diagnostics
        self.frame_count = 0
        self.start_time = time.time()

        # Create publishers (ROS 2 style)
        qos_profile = 10
        self.pub_2 = self.create_publisher(UInt32MultiArray, '/gripper_angle', qos_profile)
        self.pub_4 = self.create_publisher(UInt8MultiArray, '/servo_states', qos_profile)
        self.pub_6 = self.create_publisher(UInt8MultiArray, '/servo_states_6', qos_profile)
        self.pub_EE = self.create_publisher(UInt8MultiArray, '/error_frame_deal', qos_profile)

        # Create subscription (ROS 2 style)
        sub_qos_profile = 50
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/read_serial_data', # Subscribe to the output of the C++ driver
            self.serial_data_callback,
            sub_qos_profile)
        self.get_logger().info(f"Subscribed to {self.subscription.topic_name}")

        # Optional: Diagnostic timer (ROS 2 style)
        if self.debug_mode:
            self.timer_period = 30.0  # seconds
            self.timer = self.create_timer(self.timer_period, self.report_stats)
            self.get_logger().info(f"Reporting stats every {self.timer_period} seconds.")

    def report_stats(self):
        """Reports processing statistics - for diagnostics"""
        duration = time.time() - self.start_time
        if duration > 0:
            rate = self.frame_count / duration
            self.get_logger().info(f"Processing rate: {rate:.2f} frames/sec (Total: {self.frame_count} frames)")

    def print_hex_frame(self, frame_data):
        """Converts byte array/list to hex string and logs it."""
        if not self.debug_mode:
            return
        hex_output = " ".join([f"{byte:02X}" for byte in frame_data])
        self.get_logger().info(f"Dispatcher Rcvd Frame: {hex_output}") # Added prefix


    def serial_data_callback(self, serial_msg):
        """Callback function for processing incoming serial data frames."""
        self.frame_count += 1
        frame_data = list(serial_msg.data) # Convert immutable sequence to list

        if self.debug_mode: # Print frame received by dispatcher if debugging
            self.print_hex_frame(frame_data)

        if len(frame_data) < 2:
            self.get_logger().warn("Received data frame too short, cannot process.")
            return

        command = frame_data[1] # Command ID

        if command == 0x02: # Gripper Angle
            if len(frame_data) < 10:
                self.get_logger().warn(f"Gripper angle (0x02) frame length insufficient ({len(frame_data)} < 10)")
                return
            try:
                gripper_angle = frame_data[4] | (frame_data[5] << 8)
                extra_data_1 = frame_data[8]
                extra_data_2 = frame_data[9]
            except IndexError:
                self.get_logger().error(f"IndexError accessing gripper angle data (len={len(frame_data)})")
                return
            gripper_data_msg = UInt32MultiArray()
            gripper_data_msg.data = [gripper_angle, extra_data_1, extra_data_2]
            self.pub_2.publish(gripper_data_msg)
            if self.debug_mode:
                self.get_logger().info(f"Published Gripper Angle: {gripper_angle}, Extra: [{extra_data_1}, {extra_data_2}]")

        elif command == 0x04: # Servo States
            self.pub_4.publish(serial_msg)
            if self.debug_mode:
                self.get_logger().info("Published Servo States (0x04)")

        elif command == 0x06: # Extended Servo States
            self.pub_6.publish(serial_msg)
            if self.debug_mode:
                self.get_logger().info("Published Extended Servo States (0x06)")

        elif command == 0xEE: # Error Frame
            self.pub_EE.publish(serial_msg)
            if len(frame_data) >= 5:
                try:
                    error_type = frame_data[3]
                    error_param = frame_data[4]
                    self.get_logger().warn(f"Published Error Frame (0xEE): Type=0x{error_type:02X}, Param=0x{error_param:02X}")
                except IndexError:
                    self.get_logger().error(f"IndexError accessing error frame data (len={len(frame_data)})")
            else:
                self.get_logger().warn(f"Published short Error Frame (0xEE, len={len(frame_data)})")

        else: # Unhandled command ID
            if self.debug_mode: # Only log unhandled if debugging
                self.get_logger().warn(f"Received frame with unhandled command ID: 0x{command:02X}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialDispatcherNode()
        node.get_logger().info("Serial Dispatcher Node started. Waiting for data on /read_serial_data...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        if node: node.get_logger().fatal(f"Unhandled exception: {e}")
        else: print(f"Exception during node initialization: {e}", file=sys.stderr)
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()