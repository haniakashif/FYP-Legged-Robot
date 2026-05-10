#include "quad_interface/quad_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

// Linux hardware libraries for UART serial communication
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
#include <sstream>
#include <cstring>
#include <yaml-cpp/yaml.h>

namespace quad_interface
{

QuadHardwareInterface::~QuadHardwareInterface()
{
    stop_imu_thread_ = true;
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    if (serial_fd_ >= 0) close(serial_fd_);
    if (i2c_fd_ >= 0) close(i2c_fd_);
    if (imu_fd_ >= 0) close(imu_fd_);
}

hardware_interface::CallbackReturn QuadHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // size arrays as per number of joints
    hw_states_.resize(info_.joints.size(), 0.0);
    prev_raw_states_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    // if no filter coeffs (can just set them to 1 if we don't want filtering)
    if (fir_coeffs_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "fir_coeffs_ must contain at least one tap.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // initialize state history for filtering
    state_history_.clear();
    state_history_.resize(info_.joints.size());
    for (auto & history : state_history_) {
        history.assign(fir_coeffs_.size(), 0.0);
    }

    // open the physical UART serial port (/dev/serial0) and the USB port (/dev/ttyUSB0) on the Raspberry Pi 
    serial_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "Failed to open serial port!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    imu_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (imu_fd_ < 0) {
        RCLCPP_WARN(rclcpp::get_logger("QuadHardwareInterface"), "Failed to open IMU on /dev/ttyUSB0!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // configure the UART port for the SSC-32U (115200 baud, 8N1)
    struct termios options;
    tcgetattr(serial_fd_, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD | CS8); // 8 data bits, no parity, 1 stop bit
    tcsetattr(serial_fd_, TCSANOW, &options);

    // configure the IMU port (115200 baud, 8N1, no flow control)
    struct termios imu_options;
    tcgetattr(imu_fd_, &imu_options);
    cfsetispeed(&imu_options, B115200);
    cfsetospeed(&imu_options, B115200);
    imu_options.c_cflag |= (CLOCAL | CREAD | CS8);
    cfmakeraw(&imu_options);
    tcsetattr(imu_fd_, TCSANOW, &imu_options);

    use_adcs_ = false; // Default to safe/blind mode
    if (info_.hardware_parameters.count("use_adcs") > 0) {
        std::string use_adcs_str = info_.hardware_parameters.at("use_adcs");
        if (use_adcs_str == "true" || use_adcs_str == "True") {
            use_adcs_ = true;
        }
    }

    // Parse IMU Flags (Default to false if not found)
    use_orientation_ = false;
    use_angular_velocity_ = false;
    use_linear_acceleration_ = false;

    // which sensors to expect
    if (info_.hardware_parameters.count("use_orientation"))
        use_orientation_ = (info_.hardware_parameters.at("use_orientation") == "true" || info_.hardware_parameters.at("use_orientation") == "True");
    if (info_.hardware_parameters.count("use_angular_velocity"))
        use_angular_velocity_ = (info_.hardware_parameters.at("use_angular_velocity") == "true" || info_.hardware_parameters.at("use_angular_velocity") == "True");
    if (info_.hardware_parameters.count("use_linear_acceleration"))
        use_linear_acceleration_ = (info_.hardware_parameters.at("use_linear_acceleration") == "true" || info_.hardware_parameters.at("use_linear_acceleration") == "True");

    // if using ADCs, open the I2C bus and prepare for communication with the ADS1015 boards
    if (use_adcs_) {
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "Failed to open I2C bus!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("QuadHardwareInterface"), "ADCs enabled. I2C bus opened.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("QuadHardwareInterface"), "ADCs disabled via URDF. Running in open-loop mode.");
    }

    // if no calibration file in config
    if (info_.hardware_parameters.count("calibration_file") == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "calibration_file parameter missing from URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string calib_filepath = info_.hardware_parameters.at("calibration_file");
    
    try 
    {
        YAML::Node config = YAML::LoadFile(calib_filepath);
        YAML::Node joints_node = config["joints"];

        // ensure YAML has exactly 12 joints like the URDF
        if (joints_node.size() != info_.joints.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), 
                         "YAML joint count (%zu) does not match URDF joint count (%zu)!", 
                         joints_node.size(), info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        calibrations_.resize(info_.joints.size());
        
        // calibrations_[i] corresponds to the i-th entry in the calibrations.yaml
        for (size_t i = 0; i < joints_node.size(); ++i) {
            calibrations_[i].volt_slope     = joints_node[i]["volt_slope"].as<double>();
            calibrations_[i].volt_intercept = joints_node[i]["volt_intercept"].as<double>();
            calibrations_[i].min_rad        = joints_node[i]["min_rad"].as<double>();
            calibrations_[i].max_rad        = joints_node[i]["max_rad"].as<double>();
            calibrations_[i].min_pwm        = joints_node[i]["min_pwm"].as<int>();
            calibrations_[i].max_pwm        = joints_node[i]["max_pwm"].as<int>();
        }
        
        RCLCPP_INFO(rclcpp::get_logger("QuadHardwareInterface"), "Loaded calibration data for %zu joints successfully.", calibrations_.size());
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "Failed to load calibration YAML: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Start the dedicated IMU polling thread
    if (imu_fd_ >= 0) {
        stop_imu_thread_ = false;
        imu_thread_ = std::thread(&QuadHardwareInterface::imu_worker, this);
        RCLCPP_INFO(rclcpp::get_logger("QuadHardwareInterface"), "IMU background thread started.");
    }

    RCLCPP_INFO(rclcpp::get_logger("QuadHardwareInterface"), "Hardware initialized successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QuadHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        // for each joint's state, look at hw_states
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    // export imu states, under "um7_imu"
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "orientation.x", &imu_quat_x_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "orientation.y", &imu_quat_y_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "orientation.z", &imu_quat_z_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "orientation.w", &imu_quat_w_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "angular_velocity.x", &imu_gyro_x_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "angular_velocity.y", &imu_gyro_y_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "angular_velocity.z", &imu_gyro_z_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "linear_acceleration.x", &imu_accel_x_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "linear_acceleration.y", &imu_accel_y_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("um7_imu", "linear_acceleration.z", &imu_accel_z_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> QuadHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        // for each joint's command, look at hw_commands
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
}

void QuadHardwareInterface::imu_worker()
{
    uint8_t buffer[256];
    while (!stop_imu_thread_) {
        // Use a small timeout or non-blocking read to avoid locking the thread forever
        int bytes_read = ::read(imu_fd_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            parse_imu_buffer(buffer, bytes_read);
        } else {
            // Small sleep to prevent pegging the CPU at 100% when no data is available
            usleep(1000); 
        }
    }
}

// for parsing IMU bytes
void QuadHardwareInterface::parse_imu_buffer(uint8_t* buffer, int bytes_read)
{
    for (int i = 0; i < bytes_read; i++) {
        uint8_t byte = buffer[i];

        switch (imu_sync_state_) {
            case 0: 
                if (byte == 's') {
                    imu_sync_state_ = 1; 
                }
                break;
            case 1: 
                imu_sync_state_ = (byte == 'n') ? 2 : 0; 
                break;
            case 2: 
                imu_sync_state_ = (byte == 'p') ? 3 : 0; 
                break;
            case 3: // Packet Type
                imu_pt_ = byte;
                imu_data_len_ = ((imu_pt_ & 0x40) ? ((imu_pt_ >> 2) & 0x0F) * 4 : 4);
                
                // Safety check: UM7 max payload is usually 64 bytes. If it's higher, it's a corrupted header.
                if (imu_data_len_ > 64) {
                    imu_sync_state_ = 0;
                    break;
                }
                
                imu_sync_state_ = 4;
                break;
            case 4: // Address
                imu_addr_ = byte;
                imu_data_idx_ = 0;
                imu_calc_chksum_ = 's' + 'n' + 'p' + imu_pt_ + imu_addr_;
                imu_sync_state_ = (imu_pt_ & 0x80) ? 5 : 0; // Check has_data flag
                break;
            case 5: // Data Payload
                imu_data_buf_[imu_data_idx_++] = byte;
                imu_calc_chksum_ += byte;
                if (imu_data_idx_ >= imu_data_len_) imu_sync_state_ = 6;
                break;
            case 6: // Checksum MSB
                imu_chksum_buf_[0] = byte;
                imu_sync_state_ = 7;
                break;
            case 7: // Checksum LSB & Execute
                imu_chksum_buf_[1] = byte;
                uint16_t rx_chksum = (imu_chksum_buf_[0] << 8) | imu_chksum_buf_[1];
                
                if (rx_chksum == imu_calc_chksum_) {
                    bool is_batch = (imu_pt_ & 0x40);
                    int batch_len = (imu_pt_ >> 2) & 0x0F;

                    // Address 109: Quaternions (16-bit ints)
                    if (use_orientation_ && imu_addr_ == 109 && is_batch && batch_len >= 2) {
                        int16_t w = (imu_data_buf_[0] << 8) | imu_data_buf_[1];
                        int16_t x = (imu_data_buf_[2] << 8) | imu_data_buf_[3];
                        int16_t y = (imu_data_buf_[4] << 8) | imu_data_buf_[5];
                        int16_t z = (imu_data_buf_[6] << 8) | imu_data_buf_[7];
                        std::lock_guard<std::mutex> lock(imu_mutex_);
                        latest_imu_quat_w_ = w / 29789.09;
                        latest_imu_quat_x_ = x / 29789.09;
                        latest_imu_quat_y_ = y / 29789.09;
                        latest_imu_quat_z_ = z / 29789.09;
                    }
                    // Address 97: Processed Gyro (32-bit IEEE floats, Big-Endian)
                    else if (use_angular_velocity_ && imu_addr_ == 97 && is_batch && batch_len >= 3) {
                        uint32_t raw_x = (imu_data_buf_[0] << 24) | (imu_data_buf_[1] << 16) | (imu_data_buf_[2] << 8) | imu_data_buf_[3];
                        uint32_t raw_y = (imu_data_buf_[4] << 24) | (imu_data_buf_[5] << 16) | (imu_data_buf_[6] << 8) | imu_data_buf_[7];
                        uint32_t raw_z = (imu_data_buf_[8] << 24) | (imu_data_buf_[9] << 16) | (imu_data_buf_[10] << 8) | imu_data_buf_[11];
                        float gx, gy, gz;
                        std::memcpy(&gx, &raw_x, sizeof(float));
                        std::memcpy(&gy, &raw_y, sizeof(float));
                        std::memcpy(&gz, &raw_z, sizeof(float));

                        // Convert from degrees/s to radians/s
                        double raw_gx = gx * (M_PI / 180.0);
                        double raw_gy = gy * (M_PI / 180.0);
                        double raw_gz = gz * (M_PI / 180.0);

                        // spike clamping
                        double max_gyro_delta = 1; // 1 radian, adjust as needed
                        double delta_gx = raw_gx - latest_imu_gyro_x_;
                        double delta_gy = raw_gy - latest_imu_gyro_y_;
                        double delta_gz = raw_gz - latest_imu_gyro_z_;
                        
                        // clamp raw readings if delta exceeds threshold
                        if (std::abs(delta_gx) > max_gyro_delta) {
                            raw_gx = latest_imu_gyro_x_ + (delta_gx > 0 ? max_gyro_delta : -max_gyro_delta);
                        }
                        if (std::abs(delta_gy) > max_gyro_delta) {
                            raw_gy = latest_imu_gyro_y_ + (delta_gy > 0 ? max_gyro_delta : -max_gyro_delta);
                        }
                        if (std::abs(delta_gz) > max_gyro_delta) {
                            raw_gz = latest_imu_gyro_z_ + (delta_gz > 0 ? max_gyro_delta : -max_gyro_delta);
                        }

                        std::lock_guard<std::mutex> lock(imu_mutex_);
                        latest_imu_gyro_x_ = raw_gx;
                        latest_imu_gyro_y_ = raw_gy; 
                        latest_imu_gyro_z_ = raw_gz;
                    }
                    // Address 101 (0x65): Processed Linear Acceleration (32-bit IEEE floats, Big-Endian)
                    else if (use_linear_acceleration_ && imu_addr_ == 101 && is_batch && batch_len >= 3) {
                        uint32_t raw_x = (imu_data_buf_[0] << 24) | (imu_data_buf_[1] << 16) | (imu_data_buf_[2] << 8) | imu_data_buf_[3];
                        uint32_t raw_y = (imu_data_buf_[4] << 24) | (imu_data_buf_[5] << 16) | (imu_data_buf_[6] << 8) | imu_data_buf_[7];
                        uint32_t raw_z = (imu_data_buf_[8] << 24) | (imu_data_buf_[9] << 16) | (imu_data_buf_[10] << 8) | imu_data_buf_[11];
                        float ax, ay, az;
                        std::memcpy(&ax, &raw_x, sizeof(float));
                        std::memcpy(&ay, &raw_y, sizeof(float));
                        std::memcpy(&az, &raw_z, sizeof(float));
                        std::lock_guard<std::mutex> lock(imu_mutex_);
                        latest_imu_accel_x_ = ax; latest_imu_accel_y_ = ay; latest_imu_accel_z_ = az;
                    }
                } else {
                    // CRITICAL: If checksum fails, we likely latched onto a false 's' byte mid-payload.
                    // We must aggressively reset to avoid reading garbage.
                    imu_sync_state_ = 0;
                }
                
                imu_sync_state_ = 0;
                break;
        }
    }
}

hardware_interface::return_type QuadHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // 1. Grab the latest IMU data from the background thread safely
    if (imu_fd_ >= 0) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_quat_x_ = latest_imu_quat_x_;
        imu_quat_y_ = latest_imu_quat_y_;
        imu_quat_z_ = latest_imu_quat_z_;
        imu_quat_w_ = latest_imu_quat_w_;
        
        imu_gyro_x_ = latest_imu_gyro_x_;
        imu_gyro_y_ = latest_imu_gyro_y_;
        imu_gyro_z_ = latest_imu_gyro_z_;
        
        imu_accel_x_ = latest_imu_accel_x_;
        imu_accel_y_ = latest_imu_accel_y_;
        imu_accel_z_ = latest_imu_accel_z_;
    }

    if (!use_adcs_) // if ADCs are disabled, we skip reading and just return OK 
    {
        return hardware_interface::return_type::OK;
    }

    int addresses[3] = {0x48, 0x49, 0x4A}; // The 3 ADS1015 boards (GND, 3.3V, SDA)

    // ADS1015 Config Register MSB for Single-Ended reads on A0, A1, A2, A3
    // Configures: Single-shot mode, +/- 4.096V range, 3300 Samples Per Second
    uint8_t mux_configs[4] = {0xC3, 0xD3, 0xE3, 0xF3}; 
    uint8_t config_lsb = 0xE3; // Disables comparator, sets data rate

    // mapping between the order of ADC reads (A0-A3 for each board) and the URDF joint order in the ros2_control block/calibration.yaml/controllers,yaml
    const int adc_to_urdf_map[12] = {1, 5, 9, 0, 10, 3, 4, 6, 7, 8, 2, 11};
    int joint_index = 0;

    // Loop through the 4 channels (A0 to A3)
    for (int channel = 0; channel < 4; channel++) {

        // 1. Tell all 3 chips to start converting this channel simultaneously
        for (int board = 0; board < 3; board++) {
            ioctl(i2c_fd_, I2C_SLAVE, addresses[board]);
            uint8_t write_buf[3] = {0x01, mux_configs[channel], config_lsb};
            ::write(i2c_fd_, write_buf, 3);
        }

        // 2. Wait for the ADCs to finish (3300 SPS = ~0.3ms)
        usleep(300); 

        // 3. Collect the data from all 3 chips
        for (int board = 0; board < 3; board++) {
            ioctl(i2c_fd_, I2C_SLAVE, addresses[board]);

            bool is_ready = false;
            int timeout_counter = 0;
            uint8_t config_reg = 0x01;
            uint8_t status_buf[2];

            while (!is_ready && timeout_counter < 10) {
                ::write(i2c_fd_, &config_reg, 1);
                ::read(i2c_fd_, status_buf, 2);
                
                // Check the MSB (Bit 15). 0x80 is 10000000 in binary.
                if ((status_buf[0] & 0x80) != 0) {
                    is_ready = true;
                } else {
                    usleep(25); // Yield briefly so we don't choke the CPU, then check again
                    timeout_counter++;
                }
            }

            if (timeout_counter >= 10) {
                RCLCPP_WARN(rclcpp::get_logger("QuadHardwareInterface"), "ADC 0x%X timeout on channel %d", addresses[board], channel);
            }
            
            // Point to the Conversion Register (0x00)
            uint8_t reg = 0x00;
            ::write(i2c_fd_, &reg, 1);
            
            // Read the 2 bytes of data
            uint8_t read_buf[2];
            ::read(i2c_fd_, read_buf, 2);

            // The ADS1015 is a 12-bit ADC, but the data is left-justified in 16 bits.
            // We combine the bytes and shift right by 4.
            int16_t raw_adc = (read_buf[0] << 8) | read_buf[1];
            raw_adc = raw_adc >> 4; 

            // Math: Convert the 12-bit raw value (0-2047) to a physical Voltage.
            // 4.096V range / 2047 steps = 0.002V per step
            double voltage = raw_adc * 0.002;

            int urdf_idx = adc_to_urdf_map[joint_index];

            double raw_state = calibrations_[urdf_idx].volt_slope * voltage + calibrations_[urdf_idx].volt_intercept;

            // Spike Clamping 
            double max_delta = 0.105; // 6 degrees max change per 0.02s
            double delta = raw_state - prev_raw_states_[urdf_idx];

            if (std::abs(delta) > max_delta) 
            {
                raw_state = prev_raw_states_[urdf_idx] + (delta > 0 ? max_delta : -max_delta);
            }

            prev_raw_states_[urdf_idx] = raw_state;

            // FIR filter
            auto & history = state_history_[urdf_idx];
            if (history.size() >= fir_coeffs_.size()) {
                history.pop_back();
            }
            history.push_front(raw_state);

            double filtered_state = 0.0;
            for (size_t tap = 0; tap < fir_coeffs_.size() && tap < history.size(); ++tap) {
                filtered_state += fir_coeffs_[tap] * history[tap];
            }
            hw_states_[urdf_idx] = filtered_state;
            
            joint_index++;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type QuadHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::stringstream ssc32_command;

    // Index Order: bl_hip, br_hip, fl_hip, fr_hip, bl_knee, br_knee, fl_knee, fr_knee, bl_foot, br_foot, fl_foot, fr_foot
    const int pin_map[12] = {16, 4, 28, 0,  17, 5, 29, 1,  18, 6, 30, 2};

    for (size_t i = 0; i < hw_commands_.size(); i++) {
        auto &cal = calibrations_[i];
        double target_rad = hw_commands_[i];

        double rad_range = cal.max_rad - cal.min_rad;
        int pwm_range = cal.max_pwm - cal.min_pwm;

        int pwm_val = static_cast<int>((cal.min_pwm + cal.max_pwm)/2 + target_rad * pwm_range / rad_range);

        if (pwm_val < cal.min_pwm) pwm_val = cal.min_pwm;
        if (pwm_val > cal.max_pwm) pwm_val = cal.max_pwm;

        ssc32_command << "#" << pin_map[i] << "P" << pwm_val << " ";
    }

    ssc32_command << "T20\r";
    std::string cmd_str = ssc32_command.str();
    ::write(serial_fd_, cmd_str.c_str(), cmd_str.length());

    return hardware_interface::return_type::OK;
}

}  // namespace quad_interface

// THIS MACRO IS CRITICAL. It exposes our C++ class to the ROS 2 Controller Manager.
PLUGINLIB_EXPORT_CLASS(quad_interface::QuadHardwareInterface, hardware_interface::SystemInterface)