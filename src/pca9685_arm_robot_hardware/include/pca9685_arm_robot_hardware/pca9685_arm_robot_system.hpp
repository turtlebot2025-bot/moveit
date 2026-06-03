#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logger.hpp"

namespace pca9685_arm_robot_hardware
{

struct JointConfig
{
  int channel = 0;
  double min_pulse_us = 500.0;
  double max_pulse_us = 2500.0;
  double urdf_limit_rad = 1.57;
  double home_angle_deg = 0.0;

  double mid_pulse_us() const { return (min_pulse_us + max_pulse_us) / 2.0; }
  double half_range_us() const { return (max_pulse_us - min_pulse_us) / 2.0; }
};

class PCA9685Driver
{
public:
  PCA9685Driver(int bus_num, int address, double frequency_hz, rclcpp::Logger logger);
  ~PCA9685Driver();

  bool open();
  void close();
  bool is_open() const { return fd_ >= 0; }

  bool set_pulse_us(int channel, double pulse_us);

private:
  bool write8(uint8_t reg, uint8_t val);
  bool read8(uint8_t reg, uint8_t & val);
  bool set_frequency(double freq_hz);
  bool set_pwm_counts(int channel, int on, int off);

  int bus_num_;
  int address_;
  double frequency_hz_;
  rclcpp::Logger logger_;
  int fd_ = -1;
};

class PCA9685ArmRobotSystem : public hardware_interface::SystemInterface
{
public:
  PCA9685ArmRobotSystem();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double radians_to_pulse_us(const JointConfig & cfg, double radians) const;
  double home_radians(const JointConfig & cfg) const;

  rclcpp::Logger logger_;
  std::unique_ptr<PCA9685Driver> driver_;

  std::vector<std::string> joint_names_;
  std::vector<JointConfig> joint_cfg_;
  std::vector<double> state_pos_;
  std::vector<double> cmd_pos_;
};

}  // namespace pca9685_arm_robot_hardware

