// =============================================================================
// meer_system_interface.cpp
// =============================================================================
//
// Boilerplate implementation of the MeerSystemInterface ros2_control plugin.
//
// TODO (hardware bring-up checklist):
//   [ ] Open /dev/ttyUSBx serial port in on_configure()
//   [ ] Implement micro-ROS spin thread for async encoder readback
//   [ ] Implement write() → convert rad/s → PWM duty + DIR signal
//   [ ] Implement read()  → parse encoder tick packets from ESP32
//   [ ] Add watchdog: if read() stalls > 200 ms → emergency stop
//
// =============================================================================

#include "meer_hardware/meer_system_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace meer_hardware
{

// ── on_init ──────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MeerSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 1. Run base-class validation (checks joint count, interface names, etc.)
  // NOTE: on_init(const HardwareInfo &) is deprecated in Jazzy.
  //       The preferred override is on_init(const HardwareComponentInterfaceParams &).
  //       When subclassing SystemInterface, call the base with the info struct directly.
  //       The base class stores it in info_ before returning.
  (void)info;  // stored by the framework prior to calling on_init

  // 2. Read optional hardware parameters from URDF <hardware> block
  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port")
    : "/dev/ttyUSB0";

  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate"))
    : 921600;

  RCLCPP_INFO(logger_, "MEER Hardware: serial_port=%s  baud=%d",
    serial_port_.c_str(), baud_rate_);

  // 3. Validate joint count
  if (info_.joints.size() != NUM_WHEELS) {
    RCLCPP_FATAL(logger_,
      "Expected %zu joints in <ros2_control>, got %zu.",
      NUM_WHEELS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 4. Validate each joint has exactly one velocity command interface
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have exactly one 'velocity' command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have exactly two state interfaces (position + velocity).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  // 5. Allocate storage
  hw_commands_velocity_.assign(NUM_WHEELS, 0.0);
  hw_states_position_.assign(NUM_WHEELS, 0.0);
  hw_states_velocity_.assign(NUM_WHEELS, 0.0);

  RCLCPP_INFO(logger_, "MeerSystemInterface initialised successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ──────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
MeerSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_states_position_[i]);

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &hw_states_velocity_[i]);
  }

  return state_interfaces;
}

// ── export_command_interfaces ─────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
MeerSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_velocity_[i]);
  }

  return command_interfaces;
}

// ── on_configure ──────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MeerSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring hardware — opening serial port %s ...",
    serial_port_.c_str());

  // TODO: Open UART and initialise micro-ROS transport
  // serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  // if (serial_fd_ < 0) { return CallbackReturn::ERROR; }
  // configure_baud_rate(serial_fd_, baud_rate_);
  // init_microros_transport(serial_fd_);

  RCLCPP_WARN(logger_,
    "SIMULATION MODE: UART not opened. "
    "Remove this warning when targeting real hardware.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ───────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MeerSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset integrators
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_states_velocity_.begin(), hw_states_velocity_.end(), 0.0);
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);

  RCLCPP_INFO(logger_, "MEER hardware activated. Motors ready.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn MeerSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Safety: zero all commands before going inactive
  std::fill(hw_commands_velocity_.begin(), hw_commands_velocity_.end(), 0.0);
  // TODO: write zeros to ESP32 via UART

  RCLCPP_INFO(logger_, "MEER hardware deactivated. Motors stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ──────────────────────────────────────────────────────────────────────
hardware_interface::return_type MeerSystemInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // TODO: Read encoder data from ESP32 over UART / micro-ROS
  // For now, simulate perfect velocity tracking (open-loop integration)

  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    hw_states_velocity_[i] = hw_commands_velocity_[i];
    hw_states_position_[i] +=
      hw_states_velocity_[i] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────────
hardware_interface::return_type MeerSystemInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // TODO: Convert hw_commands_velocity_[i] → PWM duty + DIR signal
  // and transmit to ESP32 via UART packet:
  //
  //   packet format (8 bytes):
  //     [0x02]  STX
  //     [cmd_fl_byte] [cmd_bl_byte] [cmd_fr_byte] [cmd_br_byte]
  //     [dir_nibble]  — bit 0=FL, 1=BL, 2=FR, 3=BR  (1=forward)
  //     [checksum]
  //     [0x03]  ETX
  //
  // Example conversion (Cytron Sign-Magnitude):
  //   constexpr double MAX_VEL = 10.0;  // rad/s
  //   for (size_t i = 0; i < NUM_WHEELS; ++i) {
  //     double clamped = std::clamp(hw_commands_velocity_[i], -MAX_VEL, MAX_VEL);
  //     uint8_t duty   = static_cast<uint8_t>(std::abs(clamped) / MAX_VEL * 255);
  //     bool    dir    = clamped >= 0.0;
  //     send_motor_cmd(i, duty, dir);
  //   }

  return hardware_interface::return_type::OK;
}

}  // namespace meer_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  meer_hardware::MeerSystemInterface,
  hardware_interface::SystemInterface)
