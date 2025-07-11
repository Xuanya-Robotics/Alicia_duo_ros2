#include "alicia_duo_driver/alicia_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <numeric>
#include <vector>
#include <chrono>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace alicia_duo_hardware_interface
{

// Protocol Constants from the driver
constexpr uint8_t FRAME_START_BYTE = 0xAA;
constexpr uint8_t FRAME_END_BYTE = 0xFF;
constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
constexpr uint8_t FEEDBACK_GRIPPER_STATE = 0x02;
constexpr uint8_t FEEDBACK_SERVO_STATE = 0x04;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;

AliciaHardwareInterface::AliciaHardwareInterface()
  : logger_(rclcpp::get_logger("AliciaHardwareInterface"))
{
  // The constructor body can be empty
}


hardware_interface::CallbackReturn AliciaHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = info;
    port_name_ = info_.hardware_parameters["port"];
    baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
    debug_mode_ = (info_.hardware_parameters.count("debug_mode") && info_.hardware_parameters["debug_mode"] == "true");

    RCLCPP_INFO(logger_, 
        "Parameters loaded: port=%s, baud=%d, debug=%d", 
        port_name_.c_str(), baud_rate_, debug_mode_);

    initialize_maps_and_vectors();

    // Hardcoded list of expected arm joint names in their logical order.
    const std::vector<std::string> expected_arm_joints = {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    
    // Check if URDF matches expectations
    if (info_.joints.size() != expected_arm_joints.size() + 1) { // +1 for the gripper
        RCLCPP_FATAL(logger_, "URDF has %zu joints, but expected %zu arm joints + 1 gripper.", info_.joints.size(), expected_arm_joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Build the map from joint name to vector index
    for (const auto & joint : info_.joints) {
        auto it = std::find(expected_arm_joints.begin(), expected_arm_joints.end(), joint.name);
        if (it != expected_arm_joints.end()) {
            // It's an arm joint. Its index is its position in the expected list.
            size_t index = std::distance(expected_arm_joints.begin(), it);
            joint_name_to_index_map_[joint.name] = index;
        } else if (joint.name == "right_finger") {
            // It's the gripper, we handle it separately. No mapping needed.
        } else {
            RCLCPP_FATAL(logger_, "URDF contains an unexpected joint: '%s'", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // ... (rest of your init code)
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn AliciaHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "Configuring hardware interface...");
    RCLCPP_INFO(logger_, "Configuring hardware interface...");
    // Create and connect the serial communicator
    communicator_ = std::make_unique<SerialCommunicator>(port_name_, baud_rate_, debug_mode_, logger_);
    if (!communicator_->connect()) {
        // RCLCPP_FATAL(rclcpp::get_logger("AliciaHardwareInterface"), "Failed to connect to serial port %s", port_name_.c_str());
        RCLCPP_FATAL(logger_, "Failed to connect to serial port %s", port_name_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger_, "Hardware configured successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn AliciaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Reading initial robot state...");    
    // IMPORTANT: Read initial state from robot to prevent jerky motion
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    read(rclcpp::Time(0), rclcpp::Duration(0, 0)); 
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < joint_position_.size(); ++i) {
            joint_position_command_[i] = joint_position_[i];
        }
        gripper_position_command_ = gripper_position_;
    }

    RCLCPP_INFO(logger_, "Hardware interface activated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn AliciaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Deactivating hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Cleaning up hardware interface...");
    if (communicator_ && communicator_->is_connected()) {
        communicator_->disconnect();
    }
    communicator_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn AliciaHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Shutting down hardware interface...");
    if (communicator_ && communicator_->is_connected()) {
        communicator_->disconnect();
    }
    communicator_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_ERROR(logger_, "Error occurred in hardware interface. Shutting down...");
    if (communicator_ && communicator_->is_connected()) {
        communicator_->disconnect();
    }
    communicator_.reset();
    // In case of error, we transition to the "Finalized" state.
    return hardware_interface::CallbackReturn::SUCCESS; 
}






std::vector<hardware_interface::CommandInterface> AliciaHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // Iterate through the joints declared in the URDF
    for (const auto & joint : info_.joints)
    {
        if (joint.name == "right_finger") {
            // Handle the gripper directly (safe)
            command_interfaces.emplace_back(
                joint.name, "position", &gripper_position_command_);
        } else {
            // For arm joints, use the map to get the SAFE index
            size_t index = joint_name_to_index_map_.at(joint.name);
            command_interfaces.emplace_back(
                joint.name, "position", &joint_position_command_[index]);
        }
    }
    return command_interfaces;
}

std::vector<hardware_interface::StateInterface> AliciaHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    // Iterate through the joints declared in the URDF
    for (const auto & joint : info_.joints)
    {
        if (joint.name == "right_finger") {
            // Handle the gripper directly (safe)
            state_interfaces.emplace_back(
                joint.name, "position", &gripper_position_);
        } else {
            // For arm joints, use the map to get the SAFE index
            size_t index = joint_name_to_index_map_.at(joint.name);
            state_interfaces.emplace_back(
                joint.name, "position", &joint_position_[index]);
        }
    }
    return state_interfaces;
}

// ====================================================================
// The Core Logic: read() and write()
// ====================================================================

hardware_interface::return_type AliciaHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!communicator_ || !communicator_->is_connected()) {
        return hardware_interface::return_type::ERROR;
    }
    
    std::vector<uint8_t> packet;
    // Process all available packets from the serial port
    while (communicator_->get_packet(packet)) {
        if (packet.empty()) continue;

        uint8_t command_id = packet[0];
        std::vector<uint8_t> data_payload(packet.begin() + 1, packet.end());

        switch (command_id) {
            case FEEDBACK_SERVO_STATE:
                parse_servo_states_frame(data_payload);
                break;
            case FEEDBACK_GRIPPER_STATE:
                parse_gripper_state_frame(data_payload);
                break;
            case FEEDBACK_ERROR:
                parse_error_frame(data_payload);
                break;
            default:
                RCLCPP_WARN(logger_, "Received unhandled frame ID: 0x%02X", command_id);
                break;
        }
    }

    return hardware_interface::return_type::OK;
}



hardware_interface::return_type AliciaHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!communicator_ || !communicator_->is_connected()) {
        return hardware_interface::return_type::ERROR;
    }
    
    // --- Prepare and Send Servo Control Frame (0x04) ---
    {
        std::vector<uint8_t> servo_frame(SERVO_COUNT * 2 + 5, 0);
        servo_frame[0] = FRAME_START_BYTE;
        servo_frame[1] = CMD_SERVO_CONTROL;
        servo_frame[2] = SERVO_COUNT * 2; // Data payload length

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < SERVO_COUNT; ++i) {
            int joint_idx = joint_to_servo_map_index_[i];
            double direction = joint_to_servo_map_direction_[i];
            uint16_t hw_val = 2048; // Default center

            if (static_cast<size_t>(joint_idx) < joint_position_command_.size()) {
                hw_val = rad_to_hardware_value(joint_position_command_[joint_idx] * direction);
            }
            
            size_t frame_idx = 3 + i * 2;
            servo_frame[frame_idx] = hw_val & 0xFF;
            servo_frame[frame_idx + 1] = (hw_val >> 8) & 0xFF;
        }

        servo_frame[servo_frame.size() - 2] = calculate_checksum_for_write(servo_frame);
        servo_frame[servo_frame.size() - 1] = FRAME_END_BYTE;
        communicator_->write_raw_frame(servo_frame);
    }

    // --- Prepare and Send Gripper Control Frame (0x02) ---
    {
        std::vector<uint8_t> gripper_frame(8, 0);
        gripper_frame[0] = FRAME_START_BYTE;
        gripper_frame[1] = CMD_GRIPPER_CONTROL;
        gripper_frame[2] = 3; // Fixed data length
        gripper_frame[3] = 1; // Gripper ID

        std::lock_guard<std::mutex> lock(data_mutex_);
        uint16_t gripper_hw_val = rad_to_hardware_value_grip(gripper_position_command_);
        gripper_frame[4] = gripper_hw_val & 0xFF;
        gripper_frame[5] = (gripper_hw_val >> 8) & 0xFF;
        // Bytes 6 and 7 (checksum and end) are set below
        
        gripper_frame[gripper_frame.size() - 2] = calculate_checksum_for_write(gripper_frame);
        gripper_frame[gripper_frame.size() - 1] = FRAME_END_BYTE;
        communicator_->write_raw_frame(gripper_frame);
    }
    
    return hardware_interface::return_type::OK;
}

// ====================================================================
// Helper Functions (Moved from alicia_duo_driver_node.cpp)
// ====================================================================

void AliciaHardwareInterface::initialize_maps_and_vectors()
{
    // State and Command vectors
    joint_position_.resize(NUM_JOINTS, 0.0);
    joint_position_command_.resize(NUM_JOINTS, 0.0);
    gripper_position_ = 0.0;
    gripper_position_command_ = 0.0;

    // Mapping tables
    joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 2, 3, 4, 5};
    joint_to_servo_map_direction_ = {1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    servo_to_joint_map_index_ = {0, -1, 1, -1, 2, -1, 3, 4, 5};
    servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 0, 1.0, 1.0, 1.0};
}

void AliciaHardwareInterface::parse_servo_states_frame(const std::vector<uint8_t>& data_payload)
{
    if (data_payload.empty()) return;
    uint8_t data_byte_count = data_payload[0];
    if (data_payload.size() < (size_t)data_byte_count + 1) return;
    int servos_in_frame = data_byte_count / 2;

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (int i = 0; i < servos_in_frame && i < SERVO_COUNT; ++i) {
        size_t data_idx = 1 + i * 2;
        if (data_idx + 1 >= data_payload.size()) break;

        uint16_t hw_val = data_payload[data_idx] | (data_payload[data_idx + 1] << 8);
        double rad_val = hardware_value_to_rad(hw_val);

        int joint_idx = servo_to_joint_map_index_[i];
        if (joint_idx != -1 && static_cast<size_t>(joint_idx) < joint_position_.size()) {
            joint_position_[joint_idx] = rad_val * servo_to_joint_map_direction_[i];
        }
    }
}

void AliciaHardwareInterface::parse_gripper_state_frame(const std::vector<uint8_t>& data_payload)
{
    if (data_payload.size() < 4) return;
    uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
    std::lock_guard<std::mutex> lock(data_mutex_);
    gripper_position_ = hardware_value_to_rad_grip(gripper_hw_val);
}

void AliciaHardwareInterface::parse_error_frame(const std::vector<uint8_t>& payload)
{
    if (payload.size() < 2) return;
    RCLCPP_ERROR(logger_, 
                 "HW ERROR: Type=0x%02X, Param=0x%02X", payload[0], payload[1]);
}

uint8_t AliciaHardwareInterface::calculate_checksum_for_write(const std::vector<uint8_t>& frame_data)
{
    if (frame_data.size() < 4) return 0;
    const uint8_t payload_len = frame_data[2];
    if (frame_data.size() < (size_t)3 + payload_len) return 0;
    int sum = std::accumulate(frame_data.begin() + 3, frame_data.begin() + 3 + payload_len, 0);
    return static_cast<uint8_t>(sum % 2);
}

// Radian/Hardware value conversions remain the same
uint16_t AliciaHardwareInterface::rad_to_hardware_value(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
    int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
    return std::max(0, std::min(4095, value));
}

double AliciaHardwareInterface::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}

uint16_t AliciaHardwareInterface::rad_to_hardware_value_grip(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(0.0, std::min(100.0, angle_deg));
    int hardware_value = static_cast<int>(angle_deg * 8.52 + 2048.0);
    return std::max(2048, std::min(2900, hardware_value));
}

double AliciaHardwareInterface::hardware_value_to_rad_grip(uint16_t hw_value) {
    hw_value = std::max(2048, std::min(2900, (int)hw_value));
    double angle_deg = (static_cast<double>(hw_value) - 2048.0) / 8.52;
    return angle_deg * M_PI / 180.0;
}

}  // namespace alicia_duo_hardware_interface

// Export the plugin
PLUGINLIB_EXPORT_CLASS(
    alicia_duo_hardware_interface::AliciaHardwareInterface,
    hardware_interface::SystemInterface)