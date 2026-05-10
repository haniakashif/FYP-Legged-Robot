#ifndef QUAD_INTERFACE__QUAD_INTERFACE_HPP_
#define QUAD_INTERFACE__QUAD_INTERFACE_HPP_

#include <deque>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

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
  ~QuadHardwareInterface() override;

  // 2. The Memory Exporters
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 3. The Real-Time Loop Functions
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // std::vector<double> fir_coeffs_ = {0.018617559455263,-0.114628672778309,0.596290909881029,0.596290909881029,-0.114628672778309,0.018617559455263};
  std::vector<double> fir_coeffs_ = {-0.000285989470915967,	-0.000774971893404532,	-0.00168912900425785,	-0.00312184420500738,	-0.00511809225016721,	-0.00760968137531287,	-0.0103735180774238,	-0.0130085023399371,	-0.0149423203130024,	-0.0154765791888457,	-0.0138696203655593,	-0.00945131985257700,	-0.00175174646987127,	0.00937643730820774,	0.0236680560188620,	0.0404086001103375,	0.0584656014786480,	0.0763906435190855,	0.0925821587313740,	0.105486559300488,	0.113806105216014,	0.116679808754240,	0.113806105216014,	0.105486559300488,	0.0925821587313740,	0.0763906435190855,	0.0584656014786480,	0.0404086001103375,	0.0236680560188620,	0.00937643730820774,	-0.00175174646987127,	-0.00945131985257700,	-0.0138696203655593,	-0.0154765791888457,	-0.0149423203130024,	-0.0130085023399371,	-0.0103735180774238,	-0.00760968137531287,	-0.00511809225016721,	-0.00312184420500738,	-0.00168912900425785,	-0.000774971893404532,	-0.000285989470915967};
  std::vector<std::deque<double>> state_history_;

  // Memory arrays to hold the 12 joint positions
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> prev_raw_states_; // for spike clamping
  std::vector<JointCalibration> calibrations_; 

  // Memory to hold IMU data
  double imu_quat_x_ = 0.0, imu_quat_y_ = 0.0, imu_quat_z_ = 0.0, imu_quat_w_ = 1.0;
  double imu_gyro_x_ = 0.0, imu_gyro_y_ = 0.0, imu_gyro_z_ = 0.0;
  double imu_accel_x_ = 0.0, imu_accel_y_ = 0.0, imu_accel_z_ = 0.0;

  // Threading for IMU
  std::thread imu_thread_;
  std::atomic<bool> stop_imu_thread_{false};
  std::mutex imu_mutex_;
  void imu_worker();

  // Variables to hold the parsed data (protected by mutex)
  double latest_imu_quat_x_ = 0.0, latest_imu_quat_y_ = 0.0, latest_imu_quat_z_ = 0.0, latest_imu_quat_w_ = 1.0;
  double latest_imu_gyro_x_ = 0.0, latest_imu_gyro_y_ = 0.0, latest_imu_gyro_z_ = 0.0;
  double latest_imu_accel_x_ = 0.0, latest_imu_accel_y_ = 0.0, latest_imu_accel_z_ = 0.0;

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