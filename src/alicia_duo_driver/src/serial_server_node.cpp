#include "alicia_duo_driver/serial_server_node.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept> // For std::invalid_argument
#include <unistd.h>  // For access()
#include <fcntl.h>   // For O_RDWR constants with access()
#include <cstring> // For strerror
#include <cerrno>  // For errno

// Constructor (remains the same as previous correct ROS 2 version)
SerialServerNode::SerialServerNode(const rclcpp::NodeOptions & options) :
    Node("serial_server_node", options), // Node name
    is_running_(false),
    debug_mode_(false),
    baudrate_int_(921600), // Default baud rate int
    timeout_ms_(1000),     // Default read timeout (milliseconds)
    port_name_(""),
    baudrate_enum_(LibSerial::BaudRate::BAUD_DEFAULT), // Default enum
    current_port_path_("")
{
    RCLCPP_INFO(this->get_logger(), "Initializing serial_server_node...");

    // Declare and get parameters
    declareParameters();
    this->get_parameter("port", port_name_);
    this->get_parameter("baudrate", baudrate_int_);
    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("timeout_ms", timeout_ms_);

    // Validate and convert baud rate
    try {
        baudrate_enum_ = intToBaudRate(baudrate_int_);
    } catch (const std::invalid_argument& e) {
        RCLCPP_FATAL(this->get_logger(), "Unsupported baud rate parameter: %d. %s", baudrate_int_, e.what());
        RCLCPP_WARN(this->get_logger(), "Using default baud rate 921600 instead.");
        baudrate_int_ = 921600;
        baudrate_enum_ = LibSerial::BaudRate::BAUD_921600;
    }

    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), " - port: %s", port_name_.empty() ? "Not specified (REQUIRED!)" : port_name_.c_str());
    RCLCPP_INFO(this->get_logger(), " - baudrate: %d", baudrate_int_);
    RCLCPP_INFO(this->get_logger(), " - debug_mode: %s", debug_mode_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), " - timeout_ms (for read): %d", timeout_ms_);

    if (port_name_.empty()) {
         RCLCPP_ERROR(this->get_logger(), "Serial port parameter 'port' is mandatory. Please provide it (e.g., _port:=ttyUSB0).");
         // Optional: throw an exception or shutdown if port is critical for startup
    }

    // Initialize ROS Publishers and Subscribers
    pub_serial_data_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/read_serial_data", 10);
    sub_send_data_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/send_serial_data", 10, std::bind(&SerialServerNode::sendSerialDataCallback, this, _1));

    // Attempt initial connection
    if (!port_name_.empty()) {
        if (connectSerial()) {
            RCLCPP_INFO(this->get_logger(), "Initial serial connection successful.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Initial serial connection failed. Will attempt to reconnect in background.");
            // Start the reconnection timer if initial connection fails
            reconnect_timer_ = this->create_wall_timer(
                std::chrono::seconds(2), // Check every 2 seconds
                std::bind(&SerialServerNode::attemptReconnection, this));
        }
    } else {
         RCLCPP_ERROR(this->get_logger(), "Cannot attempt connection: Port parameter is empty.");
    }

    RCLCPP_INFO(this->get_logger(), "serial_server_node initialization complete.");
}

// Destructor (remains the same)
SerialServerNode::~SerialServerNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down serial_server_node...");
    stopThreadAndClosePort();
    RCLCPP_INFO(this->get_logger(), "serial_server_node has been shut down.");
}

// Declare ROS Parameters (remains the same)
void SerialServerNode::declareParameters() {
    this->declare_parameter<std::string>("port", ""); // Port name (e.g., ttyUSB0) - Made mandatory in logic
    this->declare_parameter<int>("baudrate", 921600); // Baud rate
    this->declare_parameter<bool>("debug_mode", false); // Enable hex printouts
    this->declare_parameter<int>("timeout_ms", 1000); // Read timeout
}

// Helper: Convert integer baud rate to LibSerial enum (remains the same)
LibSerial::BaudRate SerialServerNode::intToBaudRate(int baud) {
     // Add more cases if needed, based on libserial/SerialPortConstants.h
    switch (baud) {
        case 50: return LibSerial::BaudRate::BAUD_50;
        case 75: return LibSerial::BaudRate::BAUD_75;
        case 110: return LibSerial::BaudRate::BAUD_110;
        case 134: return LibSerial::BaudRate::BAUD_134;
        case 150: return LibSerial::BaudRate::BAUD_150;
        case 200: return LibSerial::BaudRate::BAUD_200;
        case 300: return LibSerial::BaudRate::BAUD_300;
        case 600: return LibSerial::BaudRate::BAUD_600;
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 1800: return LibSerial::BaudRate::BAUD_1800;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        case 460800: return LibSerial::BaudRate::BAUD_460800;
        case 500000: return LibSerial::BaudRate::BAUD_500000;
        case 576000: return LibSerial::BaudRate::BAUD_576000;
        case 921600: return LibSerial::BaudRate::BAUD_921600;
        case 1000000: return LibSerial::BaudRate::BAUD_1000000;
        case 1152000: return LibSerial::BaudRate::BAUD_1152000;
        case 1500000: return LibSerial::BaudRate::BAUD_1500000;
        case 2000000: return LibSerial::BaudRate::BAUD_2000000;
        case 2500000: return LibSerial::BaudRate::BAUD_2500000;
        case 3000000: return LibSerial::BaudRate::BAUD_3000000;
        case 3500000: return LibSerial::BaudRate::BAUD_3500000;
        case 4000000: return LibSerial::BaudRate::BAUD_4000000;
        default:
            throw std::invalid_argument("Unsupported baud rate integer value.");
    }
}

// Cleanup: Stop thread and close port (remains the same)
void SerialServerNode::stopThreadAndClosePort() {
    is_running_ = false; // Signal thread to stop

    if (reconnect_timer_) {
        reconnect_timer_->cancel();
        reconnect_timer_.reset();
        RCLCPP_INFO(this->get_logger(), "Reconnect timer stopped.");
    }

    if (read_thread_.joinable()) {
        read_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Read thread joined.");
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_.IsOpen()) {
        try {
            serial_port_.Close();
            RCLCPP_INFO(this->get_logger(), "Serial port %s closed.", current_port_path_.c_str());
            current_port_path_ = "";
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while closing serial port: %s", e.what());
        }
    }
}


// Find Serial Port (Simplified: Check permissions of specified port) (remains the same)
std::string SerialServerNode::findSerialPort() {
    if (port_name_.empty()) {
         // Already logged error in constructor if mandatory
         return "";
    }

    std::string full_port_path = "/dev/" + port_name_;

    // Check permissions using access()
    if (access(full_port_path.c_str(), R_OK | W_OK) == 0) {
        // RCLCPP_INFO(this->get_logger(), "Found specified port with R/W access: %s", full_port_path.c_str());
        return full_port_path;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot access specified port %s. Error: %s. Check permissions (user in dialout group?) or if device exists.", full_port_path.c_str(), strerror(errno));
        return "";
    }
    // Auto-detection logic is removed as libserial-dev doesn't provide list_ports easily.
}


// Connect to Serial Port (remains the same)
bool SerialServerNode::connectSerial() {
    if (serial_port_.IsOpen()) {
        RCLCPP_DEBUG(this->get_logger(), "Serial port %s is already open.", current_port_path_.c_str());
        return true;
    }

    std::string port_to_connect = findSerialPort();
    if (port_to_connect.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "No suitable serial port found or specified.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s @ %d bps", port_to_connect.c_str(), baudrate_int_);

    try {
        std::lock_guard<std::mutex> lock(serial_mutex_); // Protect serial_port_ access

        serial_port_.Open(port_to_connect); // Open the port
        serial_port_.SetBaudRate(baudrate_enum_); // Set baud rate
        // Set other parameters if needed (usually defaults are 8N1)
        // serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        // serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        // serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        // serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

        if (serial_port_.IsOpen()) {
             current_port_path_ = port_to_connect;
             RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", current_port_path_.c_str());

             // Start read thread if it's not already running
             if (!is_running_) {
                 is_running_ = true;
                 if(read_thread_.joinable()) { read_thread_.join(); } // Ensure old one finished if any
                 read_thread_ = std::thread(&SerialServerNode::readFrameThread, this);
                 RCLCPP_INFO(this->get_logger(), "Read thread started.");
             }

             // If we successfully connected, stop the reconnect timer
             if (reconnect_timer_) {
                  RCLCPP_INFO(this->get_logger(), "Connection successful, stopping reconnect timer.");
                 reconnect_timer_->cancel();
                 reconnect_timer_.reset();
             }
             return true;
        } else {
             // Should not happen if Open() didn't throw, but check anyway
             RCLCPP_ERROR(this->get_logger(), "Serial port %s IsOpen() returned false after Open() call.", port_to_connect.c_str());
             return false;
        }

    } catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", port_to_connect.c_str(), e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while connecting to serial port %s: %s", port_to_connect.c_str(), e.what());
    }

    // If connection failed, ensure port is marked as closed
    if (serial_port_.IsOpen()) {
        try {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            serial_port_.Close();
        } catch(...) {}
    }
    current_port_path_ = "";
    return false;
}

// Timer Callback: Attempt Reconnection (remains the same)
void SerialServerNode::attemptReconnection() {
    // Check if port is open (use IsOpen() which is thread-safe for read-only)
    // No need for mutex here as connectSerial will handle locking if it tries to open.
    if (!serial_port_.IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Timer attempting to reconnect to serial port...");
        connectSerial(); // connectSerial handles success/failure logging and timer cancellation
    } else {
        // This case should ideally not happen if the timer is cancelled on connect
        RCLCPP_DEBUG(this->get_logger(), "Reconnect timer triggered, but port is already open. Timer should have been cancelled.");
        if(reconnect_timer_) reconnect_timer_->cancel(); // Cancel it just in case
        reconnect_timer_.reset();
    }
}

// Send Data Callback (ROS Subscriber) (remains the same)
void SerialServerNode::sendSerialDataCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    // 1. 获取数据并进行基本验证
    // 使用 const& 避免不必要的拷贝，如果 Write 函数接受 const&
    // 但由于我们要打印它，拷贝一份可能更简单
    std::vector<uint8_t> data = msg->data;
    if (data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty data array to send, ignoring.");
        return;
    }

    // 2. 加锁以保护串口资源
    std::lock_guard<std::mutex> lock(serial_mutex_);

    // 3. 检查串口是否已打开
    if (!serial_port_.IsOpen()) {
        RCLCPP_WARN(this->get_logger(), "Attempted to send data, but serial port '%s' is not open.",
                    current_port_path_.empty() ? "unknown" : current_port_path_.c_str());
        // 不在此处尝试重连，让重连定时器负责
        return;
    }

    // 4. 尝试写入数据并处理异常
    try {
        if (debug_mode_) { // 仅在调试模式下打印尝试写入的信息
             RCLCPP_DEBUG(this->get_logger(), "Attempting to write %zu bytes to serial port %s.",
                         data.size(), current_port_path_.c_str());
        }

        // 调用 LibSerial 的 Write 函数
        // LibSerial::SerialPort::Write 通常是 void 返回类型，依赖异常来报告错误
        serial_port_.Write(data);

        // 尝试刷新输出缓冲区，确保数据尽快发送 (可选，但推荐)
        try {
            // 检查你的 LibSerial 版本文档确认函数名 (可能是 FlushOutputBuffer, FlushIOBuffers, 或 Flush)
             serial_port_.FlushOutputBuffer();
             if (debug_mode_) {
                 RCLCPP_DEBUG(this->get_logger(), "Output buffer flushed for port %s.", current_port_path_.c_str());
             }
        } catch (const std::exception& flush_e) {
            // 刷新失败通常不严重，但值得记录一个警告
            RCLCPP_WARN(this->get_logger(), "Exception during flushing output buffer for port %s: %s",
                        current_port_path_.c_str(), flush_e.what());
            // 不必因为刷新失败就关闭端口或启动重连
        }

        // 如果写入和刷新（如果尝试了）没有抛出异常，则认为发送基本成功
        // 打印发送的数据（如果启用了调试模式）
        printHexFrame(data, 0);

    // 5. 捕获并处理特定的 LibSerial 异常（如果已知）和通用异常
    } catch (const LibSerial::NotOpen& e) { // 示例：捕获端口未打开异常
        RCLCPP_ERROR(this->get_logger(), "Write error on port %s: Serial port not open: %s",
                     current_port_path_.c_str(), e.what());
        // 端口未打开，需要触发重连
        current_port_path_ = ""; // 标记路径无效
        if (!reconnect_timer_ && is_running_) { // 检查 is_running_ 避免关闭时启动
            RCLCPP_INFO(this->get_logger(), "Write failure (NotOpen): Starting reconnect timer.");
            reconnect_timer_ = this->create_wall_timer(2s, std::bind(&SerialServerNode::attemptReconnection, this));
        }
    // } catch (const LibSerial::WriteFailed& e) { // 示例：捕获写入失败异常 (如果存在)
    //     RCLCPP_ERROR(this->get_logger(), "Write error on port %s: WriteFailed: %s",
    //                  current_port_path_.c_str(), e.what());
    //     // 写入失败，很可能连接有问题，尝试关闭并重连
    //     try { serial_port_.Close(); } catch (...) {} // 尝试关闭，忽略关闭错误
    //     current_port_path_ = "";
    //     if (!reconnect_timer_ && is_running_) {
    //         RCLCPP_INFO(this->get_logger(), "Write failure (WriteFailed): Starting reconnect timer.");
    //         reconnect_timer_ = this->create_wall_timer(2s, std::bind(&SerialServerNode::attemptReconnection, this));
    //     }
    } catch (const std::exception& e) { // 捕获其他所有标准异常
        RCLCPP_ERROR(this->get_logger(), "Exception while writing to serial port %s: %s",
                     current_port_path_.c_str(), e.what());
        // 发生未知异常，最安全的做法是假定端口有问题，关闭并尝试重连
        try {
            // 确保在关闭前再次检查是否仍打开 (虽然理论上应该还是打开的)
            if(serial_port_.IsOpen()) {
                serial_port_.Close();
                 RCLCPP_INFO(this->get_logger(), "Closed port %s due to write exception.", current_port_path_.c_str());
            }
        } catch (const std::exception& close_e) {
             RCLCPP_ERROR(this->get_logger(), "Exception while closing port after write error: %s", close_e.what());
        }
        current_port_path_ = ""; // 清除记录的端口
        if (!reconnect_timer_ && is_running_) {
            RCLCPP_INFO(this->get_logger(), "Write failure (std::exception): Starting reconnect timer.");
            reconnect_timer_ = this->create_wall_timer(2s, std::bind(&SerialServerNode::attemptReconnection, this));
        }
    }
    // 锁 `lock` 在函数结束时自动释放
}

// Read Data Thread (Frame parsing logic remains the same, Checksum function calls updated below)
void SerialServerNode::readFrameThread() {
    std::vector<uint8_t> frame_buffer;
    bool wait_for_start = true;
    const size_t MAX_FRAME_LENGTH = 64; // Define max expected frame length

    RCLCPP_INFO(this->get_logger(), "Read thread started.");

    while (is_running_ && rclcpp::ok()) {
        // --- Check if port is open ---
        if (!serial_port_.IsOpen()) {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            if (!reconnect_timer_ && is_running_) {
                RCLCPP_INFO(this->get_logger(), "Read thread detected port closed. Starting reconnect timer.");
                reconnect_timer_ = this->create_wall_timer(2s, std::bind(&SerialServerNode::attemptReconnection, this));
            }
            frame_buffer.clear();
            wait_for_start = true;
            // Unlock happens automatically when lock goes out of scope
            std::this_thread::sleep_for(500ms);
            continue;
        }

        // --- Attempt to read one byte ---
        uint8_t byte = 0;
        try {
            serial_port_.ReadByte(byte, timeout_ms_);

            // --- Process the received byte (Frame Parsing Logic from ROS 1 applied here) ---
            if (wait_for_start) {
                if (byte == 0xAA) { // Start of frame
                    frame_buffer.clear();
                    frame_buffer.push_back(byte);
                    wait_for_start = false;
                }
            } else { // Building a frame
                frame_buffer.push_back(byte);

                // Check for end of frame byte (0xFF)
                // ROS 1 logic: check size >= 3 here. Let's use >= 5 for AA ADDR LEN CHK FF min valid frame
                if (byte == 0xFF && frame_buffer.size() >= 5) {
                    // Potential frame complete, check length field
                    // This calculation MUST match your protocol definition for the byte at index 2
                    uint8_t data_length_field = frame_buffer[2];
                    uint8_t expected_length = data_length_field + 5; // Assumes index 2 = data length, overhead = 5

                    if (frame_buffer.size() == expected_length) {
                        // Length matches, validate checksum using the ROS 1 logic below
                        if (serialDataCheck(frame_buffer)) {
                            // Valid frame received, publish it
                            std_msgs::msg::UInt8MultiArray msg;
                            msg.data = frame_buffer;
                            printHexFrame(frame_buffer, 1); // Log received data if debug enabled
                            pub_serial_data_->publish(msg);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Frame checksum validation failed.");
                            printHexFrame(frame_buffer, 2); // Log bad frame
                        }
                        // Reset for next frame
                        wait_for_start = true;

                    // ROS 1 had a slightly different length check condition here
                    // Let's keep the explicit MAX_FRAME_LENGTH check separate for clarity
                    // } else if (expected_length > 64 || frame_buffer.size() > 64) {
                    //     RCLCPP_WARN(this->get_logger(), "Frame error: Frame too long (expected: %d, actual: %zu, max: %zu). Discarding.",
                    //                 expected_length, frame_buffer.size(), MAX_FRAME_LENGTH);
                    //     printHexFrame(frame_buffer, 2);
                    //     wait_for_start = true; // Reset

                    } else if (frame_buffer.size() > expected_length) {
                        // Received FF, but buffer is longer than length field indicates -> corrupt frame
                        RCLCPP_WARN(this->get_logger(), "Frame error: Received FF, but frame size (%zu) > expected size (%d). Discarding.", frame_buffer.size(), expected_length);
                        printHexFrame(frame_buffer, 2);
                        wait_for_start = true; // Reset
                    }
                    // else: frame_buffer.size() < expected_length, keep receiving bytes

                } else if (frame_buffer.size() >= MAX_FRAME_LENGTH) {
                    // Frame buffer is too long, likely missed FF or corrupt data
                    RCLCPP_WARN(this->get_logger(), "Frame error: Buffer size (%zu) exceeded max length (%zu) without FF. Discarding.", frame_buffer.size(), MAX_FRAME_LENGTH);
                    printHexFrame(frame_buffer, 2);
                    wait_for_start = true; // Reset
                }
                 // else: byte is not FF or frame not long enough, continue accumulating
            } // end frame parsing logic

        } catch (const LibSerial::ReadTimeout&) {
            // Normal timeout, continue loop
            continue;
        } catch (const LibSerial::NotOpen& e) {
            // Port closed during read
             RCLCPP_WARN(this->get_logger(), "Read error: Serial port not open: %s. Will attempt to reconnect.", e.what());
             std::lock_guard<std::mutex> lock(serial_mutex_);
             current_port_path_ = "";
             if (!reconnect_timer_ && is_running_) {
                RCLCPP_INFO(this->get_logger(), "Read failure: Starting reconnect timer.");
                reconnect_timer_ = this->create_wall_timer(2s, std::bind(&SerialServerNode::attemptReconnection, this));
             }
             frame_buffer.clear();
             wait_for_start = true;
             std::this_thread::sleep_for(100ms);
        } catch (const std::exception& e) {
            // Other read exception
            RCLCPP_ERROR(this->get_logger(), "Exception during serial read: %s. Closing port and attempting to reconnect.", e.what());
            try {
                 std::lock_guard<std::mutex> lock(serial_mutex_);
                 if(serial_port_.IsOpen()) {
                     serial_port_.Close();
                 }
                 current_port_path_ = "";
                 if (!reconnect_timer_ && is_running_) {
                    RCLCPP_INFO(this->get_logger(), "Read failure: Starting reconnect timer.");
                    reconnect_timer_ = this->create_wall_timer(2s, std::bind(&SerialServerNode::attemptReconnection, this));
                 }
            } catch(...) { /* Ignore exceptions during cleanup */ }
            frame_buffer.clear();
            wait_for_start = true;
            std::this_thread::sleep_for(500ms);
        }
    } // end while loop

    RCLCPP_INFO(this->get_logger(), "Read thread finished.");
}

// --- Checksum Validation & Calculation: Copied EXACTLY from ROS 1 code provided ---

// 检查数据的校验和 (From ROS 1)
bool SerialServerNode::serialDataCheck(const std::vector<uint8_t>& data) {
    // ROS 1 used >= 4, technically correct requires >= 5 (AA ADDR LEN CHK FF)
    // but we keep ROS 1 logic here. If errors persist, consider changing to 5.
    if (data.size() < 4) {
        // Log using ROS 2 logger
        RCLCPP_WARN(this->get_logger(), "Checksum check failed: Data size too small (%zu) based on ROS 1 check (< 4).", data.size());
        return false;
    }

    // ROS 1 Checksum Calculation: sumElements(...) % 2
    uint8_t calculated_check = sumElements(data); // ROS 1 sumElements returns sum % 2
    uint8_t received_check = data[data.size() - 2]; // Checksum byte is second to last

    // Optional: Add debug log here if needed
    // RCLCPP_INFO(this->get_logger(), "Checksum check: Data size=%zu, Calculated=%u, Received=%u", data.size(), calculated_check, received_check);

    return (calculated_check == received_check);
}

// 计算数据的校验和 (From ROS 1)
// This calculates sum of bytes from index 3 up to (but not including) index size-2,
// then returns the result modulo 2 (parity check).
uint8_t SerialServerNode::sumElements(const std::vector<uint8_t>& data) {
    // ROS 1 used >= 4.
     if (data.size() < 4) {
        // Log using ROS 2 logger
        RCLCPP_ERROR(this->get_logger(), "Cannot calculate ROS 1 checksum: Data size too small (%zu < 4).", data.size());
        return 0; // Return a predictable value on error
     }

     uint32_t sum = 0;
     // Sum from index 3 (typically first data byte after LEN) up to (but not including) checksum byte
     for (size_t i = 3; i < data.size() - 2; ++i) {
        sum += data[i];
     }

     // Return sum modulo 2 (parity)
     return sum % 2;
}

// --- End of Checksum section ---


// Print Hex Frame (for debugging) (remains the same)
void SerialServerNode::printHexFrame(const std::vector<uint8_t>& data, int type) {
    if (!debug_mode_) {
        return;
    }
    std::stringstream ss;
    if (type == 0) {
        ss << "Sent (" << data.size() << " bytes): ";
    } else if (type == 1) {
        ss << "Recv OK (" << data.size() << " bytes): ";
    } else { // type == 2 for bad/discarded frames
        ss << "Recv BAD (" << data.size() << " bytes): ";
    }

    ss << std::hex << std::uppercase << std::setfill('0');
    for (const auto& byte : data) {
        ss << std::setw(2) << static_cast<int>(byte) << " ";
    }

    if (type == 2) {
        RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
}