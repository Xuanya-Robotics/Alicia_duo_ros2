#include "alicia_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace alicia_duo_hardware_interface
{

AliciaHardwareInterface::AliciaHardwareInterface()
    : gripper_angle_(0.0)
{
    // Initialize joint names
    joint_names_ = {
        "Joint1", "Joint2", "Joint3", 
        "Joint4", "Joint5", "Joint6"
    };
    
    // Initialize vectors (matching serial_comm_helper.cpp)
    filtered_angles_.resize(6, 0.0);  // 6 joints + gripper
}

AliciaHardwareInterface::~AliciaHardwareInterface()
{
    shutdownROS2Communication();
}


hardware_interface::CallbackReturn AliciaHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    // Call parent's on_init first (REQUIRED by tutorial)
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    // Validate that we have the expected number of joints
    if (info_.joints.size() != NUM_JOINTS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AliciaHardwareInterface"), 
                    "Expected %d joints, got %zu", NUM_JOINTS, info_.joints.size());
        // return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize joint vectors
    size_t num_joints = std::max(static_cast<size_t>(NUM_JOINTS), info_.joints.size());

    joint_position_.resize(NUM_JOINTS, 0.0);
    joint_position_command_.resize(NUM_JOINTS, 0.0);

    // RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "Hardware interface initialized successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool AliciaHardwareInterface::setupROS2Communication()
{
    try
    {
        // Create node
        node_ = rclcpp::Node::make_shared("alicia_hardware_interface_node");
        
        // Create publisher and subscriber (exactly like serial_comm_helper.cpp)
        joint_command_pub_ = node_->create_publisher<alicia_duo_driver::msg::ArmJointState>(
            "arm_joint_command", 10);
        
        joint_state_sub_ = node_->create_subscription<alicia_duo_driver::msg::ArmJointState>(
            "arm_joint_state", 10,
            std::bind(&AliciaHardwareInterface::feedbackCallback, this, std::placeholders::_1));
        
        // RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "ROS2 communication setup complete");
        return true;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AliciaHardwareInterface"), 
                    "Failed to setup ROS2 communication: %s", e.what());
        return false;
    }
}

// Feedback callback (exactly matching serial_comm_helper.cpp)
void AliciaHardwareInterface::feedbackCallback(const alicia_duo_driver::msg::ArmJointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Directly assign the received values to the filtered angles
    if (joint_position_.size() >= 6) {
        joint_position_[0] = msg->joint1;
        joint_position_[1] = msg->joint2;
        joint_position_[2] = msg->joint3;
        joint_position_[3] = msg->joint4;
        joint_position_[4] = msg->joint5;
        joint_position_[5] = msg->joint6;
    }

    // Gripper is already in radians (0.0 or 0.14)
    gripper_angle_ = msg->gripper;
}

// Write servo command (exactly matching serial_comm_helper.cpp)
void AliciaHardwareInterface::writeServoCommand(const std::vector<double>& joint_rad, double gripper_rad)
{
    alicia_duo_driver::msg::ArmJointState msg;

    // Convert joints from radians and populate the message fields
    msg.joint1 = joint_rad[0];
    msg.joint2 = joint_rad[1];
    msg.joint3 = joint_rad[2];
    msg.joint4 = joint_rad[3];
    msg.joint5 = joint_rad[4];
    msg.joint6 = joint_rad[5];

    // Convert gripper from radians to a binary state (0.0 or 0.14)
    msg.gripper = (gripper_rad > 0.002) ? 0.14f : 0.0f;

    // Note: time field is commented out in the message definition
    // msg.time = 0.0;

    // Publish the message
    joint_command_pub_->publish(msg);
}

// Read joint and gripper (exactly matching serial_comm_helper.cpp)
std::pair<std::vector<double>, double> AliciaHardwareInterface::readJointAndGripper()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return {joint_position_, gripper_angle_};
}

hardware_interface::return_type AliciaHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!node_) {
        RCLCPP_WARN(rclcpp::get_logger("AliciaHardwareInterface"), "Node not initialized, skipping read");
        return hardware_interface::return_type::OK;
    }

    // Process ROS2 callbacks to receive joint state updates
    rclcpp::spin_some(node_);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::cout << "Read joint positions: [";
        for (size_t i = 0; i < joint_position_.size(); ++i) {
            std::cout << joint_position_[i];
            if (i < joint_position_.size() - 1) std::cout << ", ";
        }
        std::cout << "], gripper: " << gripper_angle_ << std::endl;
    }

    // Read joint and gripper states (using the same method as serial_comm_helper)
    // auto [joints, gripper_deg] = readJointAndGripper();
    // if (joints.size() >= joint_position_.size())
    // {
    //     std::lock_guard<std::mutex> lock(data_mutex_);
    //     for (size_t i = 0; i < joint_position_.size(); ++i) {
    //         joint_position_[i] = joints[i];
    //     }
    // }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AliciaHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // Use the same write logic as serial_comm_helper.cpp
    writeServoCommand(joint_position_command_, gripper_position_command_);

    return hardware_interface::return_type::OK;
}



hardware_interface::CallbackReturn AliciaHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    if (!initializeJoints())
    {
        RCLCPP_ERROR(rclcpp::get_logger("AliciaHardwareInterface"), "Failed to initialize joints");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (!setupROS2Communication())
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AliciaHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "position", &joint_position_[i]));
                // info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AliciaHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, "position", &joint_position_command_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "Activating hardware interface...");

    // Set command interfaces to current position to avoid sudden movements
    for (size_t i = 0; i < joint_position_.size(); ++i)
    {
        joint_position_command_[i] = joint_position_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "Deactivating hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "Cleaning up hardware interface...");
    shutdownROS2Communication();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AliciaHardwareInterface"), "Shutting down hardware interface...");
    shutdownROS2Communication();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AliciaHardwareInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_ERROR(rclcpp::get_logger("AliciaHardwareInterface"), "Hardware interface encountered an error");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AliciaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AliciaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
{
    return hardware_interface::return_type::OK;
}

bool AliciaHardwareInterface::initializeJoints()
{
    std::fill(joint_position_.begin(), joint_position_.end(), 0.0);
    // std::fill(joint_velocity_.begin(), joint_velocity_.end(), 0.0);
    std::fill(joint_position_command_.begin(), joint_position_command_.end(), 0.0);
    return true;
}

void AliciaHardwareInterface::shutdownROS2Communication()
{
    if (joint_command_pub_)
    {
        joint_command_pub_.reset();
    }
    if (joint_state_sub_)
    {
        joint_state_sub_.reset();
    }
    if (node_)
    {
        node_.reset();
    }
}

}  // namespace alicia_duo_hardware_interface

// Export the plugin
PLUGINLIB_EXPORT_CLASS(
    alicia_duo_hardware_interface::AliciaHardwareInterface,
    hardware_interface::SystemInterface)