#ifndef SERIAL_SERVER_NODE_HPP
#define SERIAL_SERVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <libserial/SerialPort.h> // Use libserial-dev header
#include <libserial/SerialPortConstants.h> // For BaudRate enum
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <chrono> // For time literals and durations

// Use using for convenience
using namespace std::chrono_literals;
using std::placeholders::_1;

class SerialServerNode : public rclcpp::Node {
public:
    SerialServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SerialServerNode();

private:
    // ROS 2 Members
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_serial_data_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_send_data_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_; // Timer for reconnection attempts

    // Parameters
    std::string port_name_;      // Requested port name (e.g., "ttyUSB0")
    int baudrate_int_;       // Baud rate as integer from parameter
    bool debug_mode_;        // Debug flag
    int timeout_ms_;         // Read timeout in milliseconds

    // Serial Port Members (using libserial-dev)
    LibSerial::SerialPort serial_port_; // *** CORRECTED: Added LibSerial:: namespace ***
    LibSerial::BaudRate baudrate_enum_; // Baud rate as enum for libserial
    std::string current_port_path_; // Actual connected port (e.g., "/dev/ttyUSB0")
    std::mutex serial_mutex_;       // Mutex to protect serial port access
    std::thread read_thread_;         // Thread for reading serial data
    std::atomic<bool> is_running_;    // Flag to control the read thread

    // Methods
    void declareParameters();
    bool connectSerial();
    void sendSerialDataCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void readFrameThread();
    std::string findSerialPort(); // Simplified version for libserial-dev
    bool serialDataCheck(const std::vector<uint8_t>& data);
    uint8_t sumElements(const std::vector<uint8_t>& data); // Checksum calculation - VERIFY THIS LOGIC!
    void printHexFrame(const std::vector<uint8_t>& data, int type);
    void attemptReconnection(); // Timer callback function
    void stopThreadAndClosePort(); // Cleanup function
    LibSerial::BaudRate intToBaudRate(int baud); // Helper to convert int to enum
};

#endif // SERIAL_SERVER_NODE_HPP