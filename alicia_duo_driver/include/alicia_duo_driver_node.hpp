#ifndef ALICIA_DUO_DRIVER_NODE_HPP
#define ALICIA_DUO_DRIVER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_communicator.hpp"
#include "alicia_duo_driver/msg/arm_joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <vector>

class AliciaDuoDriverNode : public rclcpp::Node
{
public:
    AliciaDuoDriverNode();
    ~AliciaDuoDriverNode();

private:
    // Initialization
    void declare_parameters();
    void setup_ros_communications();

    // Callbacks for incoming commands
    void joint_command_callback(const alicia_duo_driver::msg::ArmJointState::SharedPtr msg);
    void zero_calibrate_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Main processing loop
    void process_serial_data();

    // Data Conversion & Framing (from arm_control_node.py)
    uint16_t rad_to_hardware_value(double angle_rad);
    uint16_t rad_to_hardware_value_grip(double angle_rad);
    uint8_t calculate_checksum(const std::vector<uint8_t>& frame_data); // Add this

    // Data Parsing (from joint_state_publisher_node.py)
    double hardware_value_to_rad(uint16_t hw_value);
    double hardware_value_to_rad_grip(uint16_t hw_value);
    void parse_servo_states_frame(const std::vector<uint8_t>& payload);
    void parse_gripper_state_frame(const std::vector<uint8_t>& payload); // Add this
    void parse_error_frame(const std::vector<uint8_t>& payload); // Add this

    std::vector<uint8_t> generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum);
    // Member Variables
    std::unique_ptr<SerialCommunicator> communicator_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    // Publishers
    rclcpp::Publisher<alicia_duo_driver::msg::ArmJointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr array_pub_;

    // Subscribers
    rclcpp::Subscription<alicia_duo_driver::msg::ArmJointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr zero_calib_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr demo_mode_sub_;

    // State
    alicia_duo_driver::msg::ArmJointState current_joint_state_;
    std::vector<double> joint_to_servo_map_direction_;
    std::vector<int> joint_to_servo_map_index_;
    std::vector<int> servo_to_joint_map_index_;
    std::vector<double> servo_to_joint_map_direction_;
    int servo_count_;
    bool debug_mode_;
    double rate_limit_sec_;
    rclcpp::Time last_process_time_;
};

#endif // ALICIA_DUO_DRIVER_NODE_HPP