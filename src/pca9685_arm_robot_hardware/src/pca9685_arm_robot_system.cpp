#include "pca9685_arm_robot_hardware/pca9685_arm_robot_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace pca9685_arm_robot_hardware
{
namespace
{
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t PRE_SCALE = 0xFE;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t ALL_LED_OFF_H = 0xFD;

constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_AI = 0x20;
constexpr uint8_t MODE1_RESTART = 0x80;

constexpr int PWM_STEPS = 4096;
constexpr double INTERNAL_OSC_HZ = 25'000'000.0;
}  // namespace

PCA9685Driver::PCA9685Driver(int bus_num, int address, double frequency_hz, rclcpp::Logger logger)
: bus_num_(bus_num), address_(address), frequency_hz_(frequency_hz), logger_(logger)
{
}

PCA9685Driver::~PCA9685Driver() { close(); }

bool PCA9685Driver::open()
{
  if (fd_ >= 0) {
    return true;
  }

  const std::string dev = "/dev/i2c-" + std::to_string(bus_num_);
  fd_ = ::open(dev.c_str(), O_RDWR);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open %s: %s", dev.c_str(), std::strerror(errno));
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    RCLCPP_ERROR(logger_, "Failed to set I2C addr 0x%02X: %s", address_, std::strerror(errno));
    close();
    return false;
  }

  // reset
  (void)write8(MODE1, MODE1_AI);
  (void)write8(MODE2, 0x04);  // totem pole
  usleep(5'000);

  if (!set_frequency(frequency_hz_)) {
    close();
    return false;
  }

  RCLCPP_INFO(logger_, "PCA9685 open OK bus=%d addr=0x%02X freq=%.1f", bus_num_, address_, frequency_hz_);
  return true;
}

void PCA9685Driver::close()
{
  if (fd_ >= 0) {
    (void)write8(ALL_LED_OFF_H, 0x10);
    ::close(fd_);
    fd_ = -1;
  }
}

bool PCA9685Driver::write8(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  const ssize_t n = ::write(fd_, buf, 2);
  if (n != 2) {
    RCLCPP_ERROR(logger_, "I2C write reg 0x%02X failed: %s", reg, std::strerror(errno));
    return false;
  }
  return true;
}

bool PCA9685Driver::read8(uint8_t reg, uint8_t & val)
{
  const ssize_t n1 = ::write(fd_, &reg, 1);
  if (n1 != 1) {
    RCLCPP_ERROR(logger_, "I2C write(reg) failed: %s", std::strerror(errno));
    return false;
  }
  const ssize_t n2 = ::read(fd_, &val, 1);
  if (n2 != 1) {
    RCLCPP_ERROR(logger_, "I2C read failed: %s", std::strerror(errno));
    return false;
  }
  return true;
}

bool PCA9685Driver::set_frequency(double freq_hz)
{
  const double prescale_f = (INTERNAL_OSC_HZ / (PWM_STEPS * freq_hz)) - 1.0;
  const int prescale = std::clamp(static_cast<int>(std::lround(prescale_f)), 3, 255);

  uint8_t old_mode = 0;
  if (!read8(MODE1, old_mode)) {
    return false;
  }

  // sleep
  if (!write8(MODE1, static_cast<uint8_t>((old_mode & 0x7F) | MODE1_SLEEP))) {
    return false;
  }
  usleep(5'000);

  if (!write8(PRE_SCALE, static_cast<uint8_t>(prescale))) {
    return false;
  }

  // wake
  if (!write8(MODE1, old_mode)) {
    return false;
  }
  usleep(5'000);

  if (!write8(MODE1, static_cast<uint8_t>(old_mode | MODE1_RESTART | MODE1_AI))) {
    return false;
  }
  usleep(5'000);
  return true;
}

bool PCA9685Driver::set_pwm_counts(int channel, int on, int off)
{
  const uint8_t base = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
  uint8_t buf[5] = {
    base,
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F),
  };
  const ssize_t n = ::write(fd_, buf, sizeof(buf));
  if (n != static_cast<ssize_t>(sizeof(buf))) {
    RCLCPP_ERROR(logger_, "I2C set_pwm ch=%d failed: %s", channel, std::strerror(errno));
    return false;
  }
  return true;
}

bool PCA9685Driver::set_pulse_us(int channel, double pulse_us)
{
  const double period_us = 1'000'000.0 / frequency_hz_;
  int off_count = static_cast<int>(std::lround(pulse_us / period_us * PWM_STEPS));
  off_count = std::clamp(off_count, 0, PWM_STEPS - 1);
  return set_pwm_counts(channel, 0, off_count);
}

PCA9685ArmRobotSystem::PCA9685ArmRobotSystem()
: logger_(rclcpp::get_logger("PCA9685ArmRobotSystem"))
{
}

hardware_interface::CallbackReturn PCA9685ArmRobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hw = info_.hardware_parameters;
  const int i2c_bus = std::stoi(hw.at("i2c_bus"));
  const int i2c_addr = std::stoi(hw.at("i2c_address"), nullptr, 0);
  const double pwm_freq = std::stod(hw.at("pwm_frequency"));

  driver_ = std::make_unique<PCA9685Driver>(i2c_bus, i2c_addr, pwm_freq, logger_);

  joint_names_.clear();
  joint_cfg_.clear();

  for (const auto & j : info_.joints) {
    joint_names_.push_back(j.name);
    JointConfig cfg;
    cfg.channel = std::stoi(j.parameters.at("channel"));
    cfg.min_pulse_us = std::stod(j.parameters.at("min_pulse_us"));
    cfg.max_pulse_us = std::stod(j.parameters.at("max_pulse_us"));
    cfg.urdf_limit_rad = std::stod(j.parameters.at("urdf_limit_rad"));
    cfg.home_angle_deg = std::stod(j.parameters.at("home_angle_deg"));
    joint_cfg_.push_back(cfg);
  }

  state_pos_.assign(joint_names_.size(), 0.0);
  cmd_pos_.assign(joint_names_.size(), 0.0);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    cmd_pos_[i] = home_radians(joint_cfg_[i]);
    state_pos_[i] = cmd_pos_[i];
    RCLCPP_INFO(
      logger_, "joint[%s] ch=%d pulse=[%.0f,%.0f] limit=±%.3f home=%.1f°",
      joint_names_[i].c_str(), joint_cfg_[i].channel, joint_cfg_[i].min_pulse_us, joint_cfg_[i].max_pulse_us,
      joint_cfg_[i].urdf_limit_rad, joint_cfg_[i].home_angle_deg);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PCA9685ArmRobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    states.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &state_pos_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> PCA9685ArmRobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.reserve(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    cmds.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
  }
  return cmds;
}

hardware_interface::CallbackReturn PCA9685ArmRobotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!driver_ || !driver_->open()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_cfg_.size(); ++i) {
    const double r = home_radians(joint_cfg_[i]);
    const double pulse = radians_to_pulse_us(joint_cfg_[i], r);
    (void)driver_->set_pulse_us(joint_cfg_[i].channel, pulse);
    cmd_pos_[i] = r;
    state_pos_[i] = r;
  }

  usleep(600'000);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PCA9685ArmRobotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (driver_ && driver_->is_open()) {
    for (size_t i = 0; i < joint_cfg_.size(); ++i) {
      const double r = home_radians(joint_cfg_[i]);
      const double pulse = radians_to_pulse_us(joint_cfg_[i], r);
      (void)driver_->set_pulse_us(joint_cfg_[i].channel, pulse);
    }
    usleep(600'000);
    driver_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PCA9685ArmRobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // No encoders: mirror command to state.
  state_pos_ = cmd_pos_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PCA9685ArmRobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!driver_ || !driver_->is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_cfg_.size(); ++i) {
    const double pulse = radians_to_pulse_us(joint_cfg_[i], cmd_pos_[i]);
    if (!driver_->set_pulse_us(joint_cfg_[i].channel, pulse)) {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

double PCA9685ArmRobotSystem::radians_to_pulse_us(const JointConfig & cfg, double radians) const
{
  const double r = std::clamp(radians, -cfg.urdf_limit_rad, cfg.urdf_limit_rad);
  return cfg.mid_pulse_us() + (r / cfg.urdf_limit_rad) * cfg.half_range_us();
}

double PCA9685ArmRobotSystem::home_radians(const JointConfig & cfg) const
{
  return cfg.home_angle_deg * M_PI / 180.0;
}

}  // namespace pca9685_arm_robot_hardware

PLUGINLIB_EXPORT_CLASS(
  pca9685_arm_robot_hardware::PCA9685ArmRobotSystem,
  hardware_interface::SystemInterface)
