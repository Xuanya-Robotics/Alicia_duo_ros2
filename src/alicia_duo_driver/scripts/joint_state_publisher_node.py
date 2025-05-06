#!/usr/bin/env python3
# coding=utf-8

"""
Joint State Publisher Node
Receives processed servo and gripper data, converts to radians,
and publishes standard ArmJointState messages.
"""

import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
from std_msgs.msg import UInt8MultiArray, UInt32MultiArray, Float32MultiArray
# Import the custom message
from alicia_duo_driver.msg import ArmJointState

# Constants
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

def u8_array_to_rad(u8_array, logger):
    """Converts raw servo byte data (2 bytes) to radians."""
    try:
        if len(u8_array) != 2:
            logger.warn(f"Servo data length error: expected 2 bytes, got {len(u8_array)}")
            return 0.0

        # Combine bytes into a 16-bit integer (Little Endian assumed from code)
        hex_value = (u8_array[0] & 0xFF) | ((u8_array[1] & 0xFF) << 8)

        # Value range check (0-4095 corresponding to 0-360 deg based on ROS1 logic)
        if hex_value < 0 or hex_value > 4095:
            logger.warn(f"Servo value out of range: {hex_value} (valid 0-4095)")
            hex_value = max(0, min(hex_value, 4095))

        # Convert to angle: -180 to +180 degrees (as per ROS1 logic)
        # 0 -> -180 deg, 2048 -> 0 deg, 4095 -> +180 deg (approx)
        angle_deg = -180.0 + (hex_value / 2048.0) * 180.0

        # Convert to radians and return
        return angle_deg * DEG_TO_RAD

    except Exception as e:
        logger.error(f"Byte to radian conversion exception: {e}")
        return 0.0

class JointStatePublisherNode(Node):
    def __init__(self):
        """Initializes the Joint State Publisher Node."""
        super().__init__('joint_state_publisher_node')

        # Declare and get parameters
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('servo_count', 9) # Number of servos expected
        self.declare_parameter('rate_limit_sec', 0.01) # Rate limit in seconds (increased default slightly)

        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        self.servo_count = self.get_parameter('servo_count').get_parameter_value().integer_value
        self.rate_limit = self.get_parameter('rate_limit_sec').get_parameter_value().double_value

        self.get_logger().info(f"JointStatePublisher Debug mode: {'enabled' if self.debug_mode else 'disabled'}")
        self.get_logger().info(f"Expected servo count: {self.servo_count}")
        self.get_logger().info(f"Rate limit: {self.rate_limit} seconds")


        # Derived configuration
        self.servo_id_min = 0
        self.servo_id_max = self.servo_count # Use servo_count param
        self.joint_count = 6 # Number of standard joints to publish

        # Data storage
        self.servo_angles_rad = np.zeros(self.servo_count, dtype=np.float32) # All servo angles in radians
        self.gripper_angle_rad = 0.0  # Gripper angle in radians
        self.but1_state = 0
        self.but2_state = 0

        # Servo to Joint Mapping (from ROS1 code)
        # Maps servo index to (joint_index, direction_multiplier)
        # None means the servo is ignored (e.g., duplicate or unused)
        self.servo_to_joint_map = {
            0: (0, 1.0),    # Servo 0 (ID 1 in some systems) -> Joint 0 (positive)
            1: None,        # Servo 1 -> Ignored
            2: (1, 1.0),    # Servo 2 -> Joint 1 (positive)
            3: None,        # Servo 3 -> Ignored (reverse of servo 2?)
            4: (2, 1.0),    # Servo 4 -> Joint 2 (positive)
            5: None,        # Servo 5 -> Ignored (reverse of servo 4?)
            6: (3, 1.0),    # Servo 6 -> Joint 3 (positive)
            7: (4, 1.0),    # Servo 7 -> Joint 4 (positive)
            8: (5, 1.0),    # Servo 8 -> Joint 5 (positive)
            # Ensure mapping covers up to self.servo_count-1 if needed
        }
        if self.servo_count > 9:
             self.get_logger().warn(f"Servo count ({self.servo_count}) > 9, but mapping only defined up to servo 8. Check servo_to_joint_map.")


        # Rate limiting timestamp
        self._last_process_time = 0.0

        # Setup ROS publishers and subscribers
        self._setup_ros_interface()

        self.get_logger().info("Joint State Publisher node initialized (publishing radians).")

    def _setup_ros_interface(self):
        """Sets up ROS 2 publishers and subscribers."""
        qos_profile = 10

        # Publisher for the custom ArmJointState message
        self.joint_state_pub = self.create_publisher(
            ArmJointState,
            '/arm_joint_state', # Topic name for standard joint states
            qos_profile)

        # Publisher for backward compatibility (Float32MultiArray)
        self.array_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_states_main', # Keep old topic name if needed
            qos_profile)

        # Subscribers to topics from the dispatcher node
        sub_qos = 10 # Can adjust QoS for subscribers if needed
        self.servo_sub = self.create_subscription(
            UInt8MultiArray,
            '/servo_states', # Topic for servo raw data (0x04)
            self.servo_states_callback,
            sub_qos)

        self.gripper_sub = self.create_subscription(
            UInt32MultiArray,
            '/gripper_angle', # Topic for gripper raw data (0x02)
            self.gripper_angle_callback,
            sub_qos)

        self.get_logger().info(f"Subscribed to {self.servo_sub.topic_name} and {self.gripper_sub.topic_name}")
        self.get_logger().info(f"Publishing to {self.joint_state_pub.topic_name} and {self.array_pub.topic_name}")


    def _should_process(self):
        """Checks if enough time has passed based on rate_limit."""
        current_time = self.get_clock().now().nanoseconds / 1e9 # Get time in seconds
        if current_time - self._last_process_time >= self.rate_limit:
            self._last_process_time = current_time
            return True
        return False

    def gripper_angle_callback(self, msg):
        """Processes incoming gripper angle data (from dispatcher)."""
        # msg is UInt32MultiArray with [angle_raw, but1, but2]
        try:
            if len(msg.data) < 3:
                self.get_logger().warn(f"Gripper angle message data too short: {len(msg.data)} < 3")
                return

            servo_value = int(msg.data[0]) # Raw value from 0x02 frame (indices 4,5 combined)
            self.but1_state = int(msg.data[1]) # Button 1 state (from index 8)
            self.but2_state = int(msg.data[2]) # Button 2 state (from index 9)

            # Gripper value range check and conversion (copied from ROS1 logic)
            # Values 2048 to 2900 map to 0 to 100 degrees
            # This seems specific, verify it's correct for your hardware!
            if servo_value < 2048 or servo_value > 2900:
                # self.get_logger().warn(f"Gripper raw value out of expected range: {servo_value} (expected 2048-2900)")
                servo_value = max(2048, min(servo_value, 2900))

            # Convert to degrees (0-100 range)
            # 8.52 = (2900 - 2048) / 100 ??? Check this constant
            angle_deg = (servo_value - 2048) / 8.52

            # Convert degrees to radians
            self.gripper_angle_rad = angle_deg * DEG_TO_RAD

            if self.debug_mode:
                self.get_logger().debug(f"Gripper raw: {servo_value}, Angle: {angle_deg:.2f} deg ({self.gripper_angle_rad:.4f} rad), BTN: [{self.but1_state}, {self.but2_state}]")

        except Exception as e:
            self.get_logger().error(f"Processing gripper angle data exception: {e}")

    def servo_states_callback(self, msg):
        """Processes incoming raw servo states data (from dispatcher)."""
        # msg is UInt8MultiArray containing the raw 0x04 frame

        # Rate limiting
        if not self._should_process():
            return

        try:
            frame_data = list(msg.data) # Convert to list

            # Basic validation (AA CMD LEN ...)
            if len(frame_data) < 3:
                self.get_logger().warn("Servo states frame data too short.")
                return

            # Check actual number of servos based on data length field (index 2)
            # Assuming LEN field (index 2) = number of data bytes = num_servos * 2
            data_byte_count = frame_data[2]
            if data_byte_count % 2 != 0:
                 self.get_logger().warn(f"Servo states data length field ({data_byte_count}) is not an even number.")
                 return

            actual_servo_count_in_frame = data_byte_count // 2

            # Validate against expected servo count parameter
            if actual_servo_count_in_frame != self.servo_count:
                self.get_logger().warn(f"Servo count mismatch in frame: Expected {self.servo_count}, Found {actual_servo_count_in_frame} based on LEN field.")
                # Decide how to proceed: return, or process only available servos?
                # Let's process based on the smaller count to avoid errors
                process_count = min(self.servo_count, actual_servo_count_in_frame)
                # return # Or adjust loop below

            else:
                process_count = self.servo_count

            # Check if frame has enough bytes for declared data length + overhead (AA CMD LEN ... CHK FF)
            min_expected_frame_len = 3 + data_byte_count + 2 # Header + Data + Footer
            if len(frame_data) < min_expected_frame_len:
                 self.get_logger().warn(f"Frame length ({len(frame_data)}) is less than expected based on LEN field ({min_expected_frame_len}). Processing cautiously.")
                 # Adjust process_count if needed to avoid reading past end of actual data
                 max_possible_servos = (len(frame_data) - 5) // 2
                 process_count = min(process_count, max_possible_servos)


            # Process servo data bytes
            temp_servo_angles = np.zeros(self.servo_count, dtype=np.float32) # Use temporary to avoid partial updates if error occurs mid-loop
            for i in range(process_count): # Iterate up to the number of servos we determined we can process
                byte_idx = 3 + i * 2 # Start index for servo i's data (after AA CMD LEN)
                if byte_idx + 1 >= len(frame_data):
                    self.get_logger().error(f"Index out of bounds trying to read servo {i} data at index {byte_idx}. Frame length {len(frame_data)}.")
                    # Stop processing this frame if indexing is wrong
                    return

                # Convert the two bytes to radians using the helper function
                servo_rad = u8_array_to_rad(frame_data[byte_idx : byte_idx + 2], self.get_logger())

                # Store if servo index is within the bounds of our array
                if i < self.servo_count:
                     temp_servo_angles[i] = servo_rad

            # If loop completed without index errors, update the main angle array
            self.servo_angles_rad = temp_servo_angles

            # Create and publish standard joint state message
            joint_state = ArmJointState()
            joint_state.header.stamp = self.get_clock().now().to_msg() # Use ROS 2 time

            # Map individual servo angles (rad) to standard joint angles (rad)
            joint_values_rad = [0.0] * self.joint_count # Initialize standard joint values

            for servo_idx, mapping in self.servo_to_joint_map.items():
                if mapping is not None and servo_idx < self.servo_count: # Check bounds
                    joint_idx, direction = mapping
                    if joint_idx < self.joint_count: # Check bounds
                         joint_values_rad[joint_idx] = self.servo_angles_rad[servo_idx] * direction
                    else:
                         self.get_logger().warn(f"Joint index {joint_idx} from mapping is out of bounds for {self.joint_count} joints.")
                # else: servo is ignored or out of expected range

            # Fill the ArmJointState message fields
            joint_state.joint1 = joint_values_rad[0]
            joint_state.joint2 = joint_values_rad[1]
            joint_state.joint3 = joint_values_rad[2]
            joint_state.joint4 = joint_values_rad[3]
            joint_state.joint5 = joint_values_rad[4]
            joint_state.joint6 = joint_values_rad[5]
            joint_state.gripper = self.gripper_angle_rad # From gripper callback
            joint_state.but1 = self.but1_state          # From gripper callback
            joint_state.but2 = self.but2_state          # From gripper callback

            # Publish the standard message
            self.joint_state_pub.publish(joint_state)

            # Publish the backward compatibility message
            compat_msg = Float32MultiArray()
            # Combine joint values and gripper value
            compat_data = joint_values_rad + [self.gripper_angle_rad]
            compat_msg.data = np.array(compat_data, dtype=np.float32).tolist() # Ensure correct type and format for msg
            self.array_pub.publish(compat_msg)

            if self.debug_mode:
                degrees = [rad * RAD_TO_DEG for rad in joint_values_rad]
                self.get_logger().debug(
                    f"Joints(deg): [{degrees[0]:.2f}, {degrees[1]:.2f}, {degrees[2]:.2f}, {degrees[3]:.2f}, {degrees[4]:.2f}, {degrees[5]:.2f}], "
                    f"Gripper(deg): {self.gripper_angle_rad * RAD_TO_DEG:.2f}, "
                    f"BTN: [{self.but1_state}, {self.but2_state}]"
                )

        except Exception as e:
            self.get_logger().error(f"Processing servo states exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointStatePublisherNode()
        node.get_logger().info("Joint State Publisher Node started.")
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