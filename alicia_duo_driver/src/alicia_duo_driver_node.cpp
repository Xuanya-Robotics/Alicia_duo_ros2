#include "alicia_duo_driver/alicia_duo_driver_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <numeric> // For std::accumulate

constexpr uint8_t CMD_SERVO_CONTROL = 0x04;
constexpr uint8_t CMD_GRIPPER_CONTROL = 0x02;
constexpr uint8_t CMD_ZERO_CAL = 0x03;
constexpr uint8_t CMD_DEMO_CONTROL = 0x13;
// Protocol Constants for feedback frames
constexpr uint8_t FEEDBACK_GRIPPER_STATE = 0x02;
constexpr uint8_t FEEDBACK_SERVO_STATE = 0x04;
constexpr uint8_t FEEDBACK_SERVO_STATE_EXT = 0x06;
constexpr uint8_t FEEDBACK_ERROR = 0xEE;


AliciaDuoDriverNode::AliciaDuoDriverNode() : Node("alicia_duo_driver_node"), last_process_time_(0, 0, RCL_ROS_TIME)
{
    declare_parameters();
    setup_ros_communications();

    // Attempt initial connection
    if (communicator_->connect()) {
        RCLCPP_INFO(this->get_logger(), "Initial connection successful. Enabling full torque mode.");
        // Send the command to disable demonstration mode (enable full torque)
        // This mimics the successful Python script's logic.
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
        communicator_->write_raw_frame(frame);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Initial connection failed. Starting reconnect timer.");
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                if (!communicator_->is_connected()) {
                    RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
                    if (communicator_->connect()) {
                        RCLCPP_INFO(this->get_logger(), "Reconnect successful! Enabling full torque mode.");
                        reconnect_timer_->cancel();
                        // Also send command after a successful reconnect
                        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
                        communicator_->write_raw_frame(frame);
                    }
                } else {
                    reconnect_timer_->cancel();
                }
            }
        );
    }
}

AliciaDuoDriverNode::~AliciaDuoDriverNode()
{
    if (communicator_) {
        communicator_->disconnect();
    }
}

void AliciaDuoDriverNode::declare_parameters()
{
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 921600);
    this->declare_parameter<bool>("debug_mode", false);
    this->declare_parameter<int>("servo_count", 9);
    this->declare_parameter<double>("rate_limit_sec", 0.01);

    std::string port = this->get_parameter("port").as_string();
    uint32_t baud_rate = this->get_parameter("baud_rate").as_int();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    servo_count_ = this->get_parameter("servo_count").as_int();
    rate_limit_sec_ = this->get_parameter("rate_limit_sec").as_double(); // Get rate limit

    communicator_ = std::make_unique<SerialCommunicator>(port, baud_rate, debug_mode_);

    // Initialize mapping tables from Python logic
    joint_to_servo_map_index_ = {0, 0, 1, 1, 2, 2, 3, 4, 5};
    joint_to_servo_map_direction_ = {1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
    servo_to_joint_map_index_ = {0, -1, 1, -1, 2, -1, 3, 4, 5}; // -1 means ignore
    servo_to_joint_map_direction_ = {1.0, 0, 1.0, 0, 1.0, 0, 1.0, 1.0, 1.0};
}

void AliciaDuoDriverNode::setup_ros_communications()
{
    // Publishers
    joint_state_pub_std_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    array_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/servo_states_main", 10);
    joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&AliciaDuoDriverNode::joint_command_callback, this, std::placeholders::_1));
        
    zero_calib_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/zero_calibrate", 10, std::bind(&AliciaDuoDriverNode::zero_calibrate_callback, this, std::placeholders::_1));
    demo_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/demonstration", 10, std::bind(&AliciaDuoDriverNode::demonstration_mode_callback, this, std::placeholders::_1));

    // Main processing timer
    processing_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // Process incoming data at ~100Hz
        std::bind(&AliciaDuoDriverNode::process_serial_data, this));
}


void AliciaDuoDriverNode::joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!communicator_->is_connected()) return;

    // Map joint names to their positions
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }

    // Define your expected joint order (update names as needed)
    std::vector<std::string> hardware_joint_names = {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };

    // Extract joint angles in hardware order
    std::vector<double> joint_angles;
    for (const auto& joint_name : hardware_joint_names) {
        auto it = joint_map.find(joint_name);
        if (it != joint_map.end()) {
            joint_angles.push_back(it->second);
        } else {
            joint_angles.push_back(0.0); // Default if not found
        }
    }

    // Extract gripper value (update name if needed)
    double gripper_value = 0.0;
    auto it_grip = joint_map.find("right_finger");
    if (it_grip != joint_map.end()) {
        gripper_value = it_grip->second * 100;
    }

    // --- Prepare and Send Servo Control Frame (0x04) ---
    {
        size_t frame_size = servo_count_ * 2 + 5;
        std::vector<uint8_t> servo_frame(frame_size);
        servo_frame[0] = FRAME_START_BYTE;
        servo_frame[1] = CMD_SERVO_CONTROL;
        servo_frame[2] = servo_count_ * 2; // Length of data payload

        for (int i = 0; i < servo_count_; ++i) {
            uint16_t hw_val = 2048; // Default to center position

            if (static_cast<size_t>(i) < joint_to_servo_map_index_.size()) {
                int joint_idx = joint_to_servo_map_index_[i];
                double direction = joint_to_servo_map_direction_[i];
                if (static_cast<size_t>(joint_idx) < joint_angles.size()) {
                    hw_val = rad_to_hardware_value(joint_angles[joint_idx] * direction);
                }
            }
            size_t frame_idx = 3 + i * 2;
            servo_frame[frame_idx] = hw_val & 0xFF;
            servo_frame[frame_idx + 1] = (hw_val >> 8) & 0xFF;
        }
        servo_frame[frame_size - 1] = FRAME_END_BYTE;
        servo_frame[frame_size - 2] = calculate_checksum(servo_frame);
        communicator_->write_raw_frame(servo_frame);
    }

    // --- Prepare and Send Gripper Control Frame (0x02) ---
    {
        std::vector<uint8_t> gripper_frame(8);
        gripper_frame[0] = FRAME_START_BYTE;
        gripper_frame[1] = CMD_GRIPPER_CONTROL;
        gripper_frame[2] = 3; // Fixed data length (ID + Low + High)
        gripper_frame[3] = 1; // Gripper ID

        uint16_t gripper_hw_val = rad_to_hardware_value_grip(gripper_value);
        gripper_frame[4] = gripper_hw_val & 0xFF;
        gripper_frame[5] = (gripper_hw_val >> 8) & 0xFF;

        gripper_frame[7] = FRAME_END_BYTE;
        gripper_frame[6] = calculate_checksum(gripper_frame);

        communicator_->write_raw_frame(gripper_frame);
    }
}



uint16_t AliciaDuoDriverNode::rad_to_hardware_value_grip(double angle_rad)
{
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(0.0, std::min(100.0, angle_deg)); // Clamp to expected [0, 100] deg range
    // Linear map [0, 100] deg to [2048, 2900]
    int hardware_value = static_cast<int>(angle_deg * 8.52 + 2048.0);
    return std::max(2048, std::min(2900, hardware_value));
}

uint16_t AliciaDuoDriverNode::rad_to_hardware_value(double angle_rad) {
    double angle_deg = angle_rad * 180.0 / M_PI;
    angle_deg = std::max(-180.0, std::min(180.0, angle_deg));
    int value = static_cast<int>((angle_deg + 180.0) / 360.0 * 4096.0);
    return std::max(0, std::min(4095, value));
}


uint8_t AliciaDuoDriverNode::calculate_checksum(const std::vector<uint8_t>& frame_data)
{
    // The checksum is the sum of the DATA PAYLOAD bytes, modulo 2.
    // The frame_data vector is passed in *before* the checksum is calculated and inserted.
    // The payload starts at index 3 and its length is specified at index 2.
    if (frame_data.size() < 4) {
        return 0; // Frame is too short to have a payload
    }
    
    // The length of the actual data payload.
    const uint8_t payload_len = frame_data[2];

    // Ensure the frame is large enough to contain the declared payload
    if (frame_data.size() < (size_t)3 + payload_len) {
        return 0; 
    }

    // Sum from the beginning of the payload (index 3) for the length of the payload.
    int sum = std::accumulate(frame_data.begin() + 3, 
                              frame_data.begin() + 3 + payload_len, 
                              0);

    return static_cast<uint8_t>(sum % 2);
}

std::vector<uint8_t> AliciaDuoDriverNode::generate_simple_frame(uint8_t command, uint8_t data, bool use_checksum)
{
    std::vector<uint8_t> frame(6);
    frame[0] = FRAME_START_BYTE;
    frame[1] = command;
    frame[2] = 0x01; // Data length is always 1 for these simple frames
    frame[3] = data & 0xFF; // Data byte

    if (use_checksum) {
        // Checksum is just the data byte modulo 2, as per the Python logic
        frame[4] = data % 2;
    } else {
        frame[4] = 0x00;
    }

    frame[5] = FRAME_END_BYTE;
    return frame;
}

void AliciaDuoDriverNode::zero_calibrate_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Received Zero Calibration command.");
        auto frame = generate_simple_frame(CMD_ZERO_CAL, 0x00, false);
        communicator_->write_raw_frame(frame);
    }
}

void AliciaDuoDriverNode::demonstration_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Enabling Demonstration Mode (Zero Torque).");
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x00, false);
        communicator_->write_raw_frame(frame);
    } else {
        RCLCPP_INFO(this->get_logger(), "Disabling Demonstration Mode (Full Torque).");
        auto frame = generate_simple_frame(CMD_DEMO_CONTROL, 0x01, true);
        communicator_->write_raw_frame(frame);
    }

}


void AliciaDuoDriverNode::process_serial_data()
{
    if (!communicator_->is_connected()) return;

    std::vector<uint8_t> packet;
    // Process all available packets in the queue
    while (communicator_->get_packet(packet)) {
        if (packet.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty packet.");
            continue;
        }

        // The first byte of the payload is the command ID
        uint8_t command_id = packet[0];

        // Create a sub-vector that contains the actual data payload for the command
        std::vector<uint8_t> data_payload;
        if (packet.size() > 1) {
            data_payload.assign(packet.begin() + 1, packet.end());
        }

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
            case FEEDBACK_SERVO_STATE_EXT:
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Received unhandled Extended Servo State frame (0x06)");
                }
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Received frame with unhandled command ID: 0x%02X", command_id);
                break;
        }
    }
}

void AliciaDuoDriverNode::parse_servo_states_frame(const std::vector<uint8_t>& data_payload)
{
    // Rate Limiting
    auto now = this->get_clock()->now();
    if ((now - last_process_time_).seconds() < rate_limit_sec_) {
        return; // Skip processing if too soon
    }
    last_process_time_ = now;

    // The data_payload contains the data bytes *after* the command ID.
    // The first byte of this payload is the data length.
    if (data_payload.empty()) {
        RCLCPP_WARN(this->get_logger(), "Servo state data payload is empty.");
        return;
    }
    uint8_t data_byte_count = data_payload[0];
    if (data_payload.size() < (size_t)data_byte_count + 1) {
        RCLCPP_WARN(this->get_logger(), "Servo state data payload is shorter than its own length field indicates.");
        return;
    }
    int servos_in_frame = data_byte_count / 2;

    // Data Processing & State Update
    std::vector<double> joint_values(6, 0.0);
    for (int i = 0; i < servos_in_frame && i < servo_count_; ++i) {
        size_t data_idx = 1 + i * 2; // Data starts after the length byte.
        if (data_idx + 1 >= data_payload.size()) break; // Bounds check
        uint16_t hw_val = data_payload[data_idx] | (data_payload[data_idx + 1] << 8);
        double rad_val = hardware_value_to_rad(hw_val);

        if (static_cast<size_t>(i) < servo_to_joint_map_index_.size()) {
            int joint_idx = servo_to_joint_map_index_[i];
            if (joint_idx != -1) {
                joint_values[joint_idx] = rad_val * servo_to_joint_map_direction_[i];
            }
        }
    }

    // Publish Unified State
    current_joint_state_.header.stamp = now;
    current_joint_state_.joint1 = joint_values[0];
    current_joint_state_.joint2 = joint_values[1];
    current_joint_state_.joint3 = joint_values[2];
    current_joint_state_.joint4 = joint_values[3];
    current_joint_state_.joint5 = joint_values[4];
    current_joint_state_.joint6 = joint_values[5];
    // joint_state_pub_->publish(current_joint_state_);



    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now;
    js_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    js_msg.position = joint_values;
    joint_state_pub_std_->publish(js_msg);
    

    // Publish Backward Compatibility Message
    std_msgs::msg::Float32MultiArray compat_msg;
    compat_msg.data = {(float)joint_values[0], (float)joint_values[1], (float)joint_values[2], (float)joint_values[3], (float)joint_values[4], (float)joint_values[5], (float)current_joint_state_.gripper};
    array_pub_->publish(compat_msg);
}

void AliciaDuoDriverNode::parse_gripper_state_frame(const std::vector<uint8_t>& data_payload)
{
    // The data_payload is what comes *after* the command ID (0x02).
    // The structure is [LEN, ID, Low, High, ?, ?, BTN1, BTN2].
    // The warning log shows the payload size is 8 bytes.
    if (data_payload.size() < 8) {
        RCLCPP_WARN(this->get_logger(), "Gripper state data payload is smaller than the expected 8 bytes: %zu bytes", data_payload.size());
        return;
    }

    uint16_t gripper_hw_val = data_payload[2] | (data_payload[3] << 8);
    current_joint_state_.gripper = hardware_value_to_rad_grip(gripper_hw_val);

    // Button states are the last two bytes of this payload.
    // We don't need bitwise operations if the hardware sends 0 or 1 directly in each byte.
    current_joint_state_.button1 = data_payload[6];
    current_joint_state_.button2 = data_payload[7];
}


void AliciaDuoDriverNode::parse_error_frame(const std::vector<uint8_t>& payload)
{
    // Based on Python: error_type at index 3 -> 1 of command payload
    // error_param at index 4 -> 2 of command payload
    if (payload.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Error frame payload too short: %zu bytes", payload.size());
        return;
    }
    uint8_t error_type = payload[0];
    uint8_t error_param = payload[1];
    RCLCPP_ERROR(this->get_logger(), "Received Error Frame from Hardware: Type=0x%02X, Param=0x%02X", error_type, error_param);
}


double AliciaDuoDriverNode::hardware_value_to_rad(uint16_t hw_value) {
    hw_value = std::max(0, std::min(4095, (int)hw_value));
    double angle_deg = -180.0 + (static_cast<double>(hw_value) / 4095.0) * 360.0;
    return angle_deg * M_PI / 180.0;
}

double AliciaDuoDriverNode::hardware_value_to_rad_grip(uint16_t hw_value) {
    hw_value = std::max(2048, std::min(2900, (int)hw_value));
    // Linear map [2048, 2900] back to [0, 100] deg
    double angle_deg = (static_cast<double>(hw_value) - 2048.0) / 8.52;
    return angle_deg * M_PI / 180.0;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AliciaDuoDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}