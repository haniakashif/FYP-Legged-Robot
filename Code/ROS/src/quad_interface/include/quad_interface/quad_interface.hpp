#ifndef QUAD_INTERFACE__QUAD_INTERFACE_HPP_
#define QUAD_INTERFACE__QUAD_INTERFACE_HPP_

#include <vector>
#include <string>

// Core ROS 2 and Hardware Interface libraries
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace quad_interface
{

struct JointCalibration {
  // Potentiometer Calibration: Angle = (slope * voltage) + intercept
  double volt_slope;
  double volt_intercept;

  // Actuator Calibration: Mapping Radians to PWM
  double min_rad;
  double max_rad;
  int min_pwm;
  int max_pwm;
};

class QuadHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // 1. The Setup Function
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // 2. The Memory Exporters
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 3. The Real-Time Loop Functions
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Memory arrays to hold the 12 joint positions
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<JointCalibration> calibrations_; 

  // Memory to hold IMU data
  double imu_quat_x_ = 0.0, imu_quat_y_ = 0.0, imu_quat_z_ = 0.0, imu_quat_w_ = 1.0;
  double imu_gyro_x_ = 0.0, imu_gyro_y_ = 0.0, imu_gyro_z_ = 0.0;
  double imu_accel_x_ = 0.0, imu_accel_y_ = 0.0, imu_accel_z_ = 0.0;

  // File descriptor for the Raspberry Pi's physical UART serial port
  int serial_fd_;
  int i2c_fd_;
  int imu_fd_;

  // Flag to indicate what kind of feedback is expected for the read() function
  bool use_adcs_;
  bool use_orientation_;
  bool use_angular_velocity_;
  bool use_linear_acceleration_;

  // UM7 Serial Parser State Machine and Related Variables
  void parse_imu_buffer(uint8_t* buffer, int bytes_read);
  int imu_sync_state_ = 0;
  uint8_t imu_pt_ = 0;
  uint8_t imu_addr_ = 0;
  int imu_data_len_ = 0;
  int imu_data_idx_ = 0;
  uint8_t imu_data_buf_[64];
  uint8_t imu_chksum_buf_[2];
  uint16_t imu_calc_chksum_ = 0;
};

}  // namespace quad_interface

#endif  // QUAD_INTERFACE__QUAD_INTERFACE_HPP_