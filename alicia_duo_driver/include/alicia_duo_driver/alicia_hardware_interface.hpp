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
#include "alicia_duo_driver/msg/arm_joint_state.hpp"

// Add serial driver includes
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_communicator.hpp"

namespace alicia_duo_hardware_interface
{

class AliciaHardwareInterface : public hardware_interface::SystemInterface
{
public:
    AliciaHardwareInterface();
    virtual ~AliciaHardwareInterface() = default;

    // Standard ros2_control lifecycle methods
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    // Methods for providing state and command interfaces
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Methods for reading from and writing to the hardware
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // == Hardware Communication ==
    std::unique_ptr<SerialCommunicator> communicator_;

    // == Configuration (from URDF) ==
    std::string port_name_;
    uint32_t baud_rate_;
    bool debug_mode_;

    // == State and Command Vectors (for ros2_control) ==
    static constexpr int NUM_JOINTS = 6;
    static constexpr int SERVO_COUNT = 9;
    std::vector<double> joint_position_;
    std::vector<double> joint_position_command_;
    double gripper_position_;
    double gripper_position_command_;
    std::mutex data_mutex_;



    // == Protocol and Mapping (from the old driver node) ==
    std::vector<int> joint_to_servo_map_index_;
    std::vector<double> joint_to_servo_map_direction_;
    std::vector<int> servo_to_joint_map_index_;
    std::vector<double> servo_to_joint_map_direction_;
    std::unordered_map<std::string, size_t> joint_name_to_index_map_;

    // == Helper methods moved from the old driver node ==
    void initialize_maps_and_vectors();
    void parse_servo_states_frame(const std::vector<uint8_t>& data_payload);
    void parse_gripper_state_frame(const std::vector<uint8_t>& data_payload);
    void parse_error_frame(const std::vector<uint8_t>& payload);
    uint16_t rad_to_hardware_value(double angle_rad);
    uint16_t rad_to_hardware_value_grip(double angle_rad);
    double hardware_value_to_rad(uint16_t hw_value);
    double hardware_value_to_rad_grip(uint16_t hw_value);
    uint8_t calculate_checksum_for_write(const std::vector<uint8_t>& frame_data);
    rclcpp::Logger logger_;

};

}  // namespace alicia_duo_hardware_interface

#endif  // ALICIA_HARDWARE_INTERFACE_HPP