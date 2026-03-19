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
#include <yaml-cpp/yaml.h>

namespace quad_interface
{

hardware_interface::CallbackReturn QuadHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 1. Size our memory arrays to match the 12 joints defined in the URDF
    hw_states_.resize(info_.joints.size(), 0.0);
    hw_commands_.resize(info_.joints.size(), 0.0);

    // 2. Open the physical UART serial port on the Raspberry Pi (/dev/serial0)
    serial_fd_ = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "Failed to open serial port!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 3. Configure the UART port for the SSC-32U (115200 baud, 8N1)
    struct termios options;
    tcgetattr(serial_fd_, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD | CS8); // 8 data bits, no parity, 1 stop bit
    tcsetattr(serial_fd_, TCSANOW, &options);

    use_adcs_ = false; // Default to safe/blind mode
    if (info_.hardware_parameters.count("use_adcs") > 0) {
        std::string use_adcs_str = info_.hardware_parameters.at("use_adcs");
        if (use_adcs_str == "true" || use_adcs_str == "True") {
            use_adcs_ = true;
        }
    }

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

    if (info_.hardware_parameters.count("calibration_file") == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), "calibration_file parameter missing from URDF!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string calib_filepath = info_.hardware_parameters.at("calibration_file");
    
    try {
        YAML::Node config = YAML::LoadFile(calib_filepath);
        YAML::Node joints_node = config["joints"];

        // Safety check: Ensure YAML has exactly 12 joints like the URDF
        if (joints_node.size() != info_.joints.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("QuadHardwareInterface"), 
                         "YAML joint count (%zu) does not match URDF joint count (%zu)!", 
                         joints_node.size(), info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        calibrations_.resize(info_.joints.size());

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

hardware_interface::return_type QuadHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    if (!use_adcs_) // If ADCs are disabled, we skip reading and just return OK 
    {
        return hardware_interface::return_type::OK;
    }

    int addresses[3] = {0x48, 0x49, 0x4A}; // The 3 ADS1015 boards

    // ADS1015 Config Register MSB for Single-Ended reads on A0, A1, A2, A3
    // Configures: Single-shot mode, +/- 4.096V range, 3300 Samples Per Second
    uint8_t mux_configs[4] = {0xC3, 0xD3, 0xE3, 0xF3}; 
    uint8_t config_lsb = 0xE3; // Disables comparator, sets data rate

    int joint_index = 0;

    // Loop through the 4 channels (A0 to A3)
    for (int channel = 0; channel < 4; channel++) {

        // 1. Tell all 3 chips to start converting this channel simultaneously
        for (int board = 0; board < 3; board++) {
            ioctl(i2c_fd_, I2C_SLAVE, addresses[board]);
            uint8_t write_buf[3] = {0x01, mux_configs[channel], config_lsb};
            ::write(i2c_fd_, write_buf, 3);
        }

        // 2. Wait for the ADCs to finish (3300 SPS = ~0.3ms. We wait 0.5ms to be safe)
        usleep(500); 

        // 3. Collect the data from all 3 chips
        for (int board = 0; board < 3; board++) {
            ioctl(i2c_fd_, I2C_SLAVE, addresses[board]);
            
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

            // TODO for Point 2: Convert 'voltage' to 'radians' using your custom motor bounds!
            hw_states_[joint_index] = calibrations_[joint_index].volt_slope * voltage + calibrations_[joint_index].volt_intercept;
            
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