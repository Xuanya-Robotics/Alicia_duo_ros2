#ifndef ALICIA_HARDWARE_INTERFACE_HPP
#define ALICIA_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <unordered_map>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Include the generated message header for ROS2
#include "alicia_duo_driver/msg/arm_joint_state.hpp"

namespace alicia_duo_hardware_interface
{

class AliciaHardwareInterface : public hardware_interface::SystemInterface
{
public:
    AliciaHardwareInterface();
    virtual ~AliciaHardwareInterface();

    // Lifecycle interface methods
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    // Export methods - these are now automatic in newer ROS2 versions
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // Optional command mode switching
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces) override;

    void set_joint_commands(const std::vector<double>& commands) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // The first 6 commands are for the joints, the rest are for the gripper
        joint_position_command_.resize(NUM_JOINTS, 0.0);
        if (commands.size() >= NUM_JOINTS) {
            for (size_t i = 0; i < NUM_JOINTS; ++i) {
                joint_position_command_[i] = commands[i];
            }
            gripper_position_command_ = (commands.size() > NUM_JOINTS) ? commands[NUM_JOINTS] : 0.0;
        }
    }

    std::vector<double> joint_position_;

private:
    // Hardware info
    hardware_interface::HardwareInfo info_;
    
    // ROS2 node and communication
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<alicia_duo_driver::msg::ArmJointState>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<alicia_duo_driver::msg::ArmJointState>::SharedPtr joint_state_sub_;
    
    // Joint configuration
    static constexpr int NUM_JOINTS = 6;
    std::vector<std::string> joint_names_;
    
    // State vectors (matching serial_comm_helper.cpp naming)
    std::vector<double> filtered_angles_;  // Joint positions from feedback
    double gripper_angle_ = 0.0;          // Gripper position from feedback
    
    // These are for the hardware interface
    std::vector<double> joint_position_command_;
    double gripper_position_command_ = 0.0;

    // Thread safety
    std::mutex data_mutex_;  // Matching serial_comm_helper naming

    // Callback function (matching serial_comm_helper.cpp)
    void feedbackCallback(const alicia_duo_driver::msg::ArmJointState::SharedPtr msg);

    // Helper methods
    bool initializeJoints();
    bool setupROS2Communication();
    void shutdownROS2Communication();
    
    // Methods matching serial_comm_helper functionality
    void writeServoCommand(const std::vector<double>& joint_rad, double gripper_rad);
    std::pair<std::vector<double>, double> readJointAndGripper();
};

}  // namespace alicia_duo_hardware_interface

#endif  // ALICIA_HARDWARE_INTERFACE_HPP