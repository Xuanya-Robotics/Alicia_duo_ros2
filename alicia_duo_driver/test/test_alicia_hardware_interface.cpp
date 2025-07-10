#include "alicia_duo_driver/alicia_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "alicia_duo_driver/msg/arm_joint_state.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a single hardware interface instance for the test
    auto hw = std::make_unique<alicia_duo_hardware_interface::AliciaHardwareInterface>();
    
    // Create hardware info for 6 joints
    hardware_interface::HardwareInfo info;
    info.name = "alicia_duo_robot";
    info.type = "system";
    for (int i = 1; i <= 6; ++i) {
        hardware_interface::ComponentInfo joint;
        joint.name = "joint" + std::to_string(i);
        joint.type = "joint";
        hardware_interface::InterfaceInfo pos_interface;
        pos_interface.name = "position";
        joint.state_interfaces.push_back(pos_interface);
        info.joints.push_back(joint);
    }

    // Initialize and configure the hardware interface once
    if (hw->on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to initialize hardware interface");
        return -1;
    }
    if (hw->on_configure(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to configure hardware interface");
        return -1;
    }
    hw->on_activate(rclcpp_lifecycle::State());

    rclcpp::Rate rate(10);  // 10 Hz
    std::vector<double> zero_commands(7, 0.0);
    while (rclcpp::ok()) {
        hw->read(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.1));

        // Now read the updated positions
        const auto& positions = hw->joint_position_;
        std::cout << "---" << std::endl;
        for (size_t i = 0; i < positions.size(); ++i) {
            std::cout << "Joint" << (i + 1) << " position: " << positions[i] << std::endl;
        }

        hw->set_joint_commands(zero_commands);
        hw->write(rclcpp::Clock().now(), rclcpp::Duration::from_seconds(0.1));


        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}