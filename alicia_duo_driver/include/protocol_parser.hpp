#ifndef PROTOCOL_PARSER_HPP
#define PROTOCOL_PARSER_HPP

#include <vector>
#include <string>
#include <cstdint>

// A structure to hold the parsed state of the robot
struct RobotState {
    std::vector<double> joint_positions; // 6 joints in radians
    double gripper_position; // in radians
    int button1_state;
    int button2_state;
};

class ProtocolParser
{
public:
    ProtocolParser();

    // Parses a complete, validated frame and updates the state
    bool parse_frame(const std::vector<uint8_t>& frame, RobotState& state);

    // Formats a command for all joints and the gripper
    std::vector<uint8_t> format_servo_command(const std::vector<double>& joint_rads, double gripper_rad);
    
    // Formats simple commands like zero calibration or demo mode
    std::vector<uint8_t> format_simple_command(uint8_t command_id, uint8_t data = 0x00, bool use_checksum = false);

    // Validates a raw frame from the serial port
    bool validate_frame(const std::vector<uint8_t>& buffer, std::vector<uint8_t>& frame);

private:
    // Constants
    static constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    static constexpr uint8_t FRAME_HEADER = 0xAA;
    static constexpr uint8_t FRAME_FOOTER = 0xFF;

    // Helper methods ported from Python
    uint8_t calculate_checksum(const std::vector<uint8_t>& frame);
    int rad_to_hardware_value(double angle_rad);
    int rad_to_hardware_value_grip(double angle_rad);
    double hardware_value_to_rad(int hex_value);
    double hardware_value_to_rad_grip(int servo_value);
};

#endif // PROTOCOL_PARSER_HPP