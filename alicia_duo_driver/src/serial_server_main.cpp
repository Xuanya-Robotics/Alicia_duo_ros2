#include "alicia_duo_driver/serial_server_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // Initialize ROS 2

    // Create node options, allows parameters to be passed via command line or launch file
    rclcpp::NodeOptions options;
    //options.automatically_declare_parameters_from_overrides(true);

    // Create the SerialServerNode instance
    auto serial_server_node = std::make_shared<SerialServerNode>(options);

    // Spin the node so callbacks can be processed
    rclcpp::spin(serial_server_node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}