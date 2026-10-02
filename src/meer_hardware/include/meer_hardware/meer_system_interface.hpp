// =============================================================================
// meer_system_interface.hpp
// =============================================================================
//
// ros2_control SystemInterface for the MEER Rover Hardware
// ─────────────────────────────────────────────────────────
//
// PURPOSE
//   Bridges ros2_control's controller_manager with the physical drive system:
//
//     controller_manager
//         │  velocity commands (rad/s)
//         ▼
//   MeerSystemInterface          ← this class
//         │  UART / micro-ROS
//         ▼
//       ESP32 (micro-ROS agent)
//         │  PWM + DIR signals
//         ▼
//   4× Cytron MDDS30 motor drivers (Sign-Magnitude mode)
//         │
//         ▼
//   4× 24 V DC motors (front/back × left/right)
//
// COMMUNICATION PROTOCOL (to be implemented in meer_system_interface.cpp)
//   - Transport : UART (/dev/ttyUSB0, 921600 baud) or USB-CDC
//   - Protocol  : micro-ROS over serial (rmw_microxrcedds)
//   - Topics published TO the ESP32:
//       /wheel_cmd  [std_msgs/Float32MultiArray]  — [FLW, BLW, FRW, BRW] rad/s
//   - Topics subscribed FROM the ESP32:
//       /wheel_enc  [std_msgs/Float32MultiArray]  — encoder velocities rad/s
//       /wheel_pos  [std_msgs/Float32MultiArray]  — integrated positions rad
//
// CYTRON MDDS30 WIRING (Sign-Magnitude PWM+DIR)
//   Each channel: DIR pin (GPIO) + PWM pin (LEDC timer, 10-bit, 20 kHz)
//   duty = abs(cmd_velocity) / max_velocity × 1023
//   dir  = (cmd_velocity >= 0) ? HIGH : LOW
//
// =============================================================================
#ifndef MEER_HARDWARE__MEER_SYSTEM_INTERFACE_HPP_
#define MEER_HARDWARE__MEER_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace meer_hardware
{

/// Number of driven wheels
static constexpr std::size_t NUM_WHEELS = 4;

/// Joint index constants (matches controllers.yaml wheel order)
enum WheelIndex : std::size_t
{
  FRONT_LEFT  = 0,
  BACK_LEFT   = 1,
  FRONT_RIGHT = 2,
  BACK_RIGHT  = 3,
};

class MeerSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MeerSystemInterface)

  // ── Lifecycle ─────────────────────────────────────────────────────────

  /// Called once by controller_manager to validate URDF <ros2_control> block.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /// Export read-only state interfaces (position, velocity per wheel).
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// Export write-only command interfaces (velocity per wheel).
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// Opens UART / micro-ROS agent connection.
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Activates motor drivers; sets initial PWM to 0.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Sends zero velocity before deactivation (safety stop).
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Reads encoder positions and velocities from ESP32.
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /// Converts velocity commands → PWM+DIR and writes to ESP32.
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ── Internal state storage ─────────────────────────────────────────────

  /// Commanded velocity for each wheel [rad/s].
  std::vector<double> hw_commands_velocity_;

  /// Measured wheel positions [rad] (integrated from encoder ticks).
  std::vector<double> hw_states_position_;

  /// Measured wheel velocities [rad/s].
  std::vector<double> hw_states_velocity_;

  /// Joint names in order matching WheelIndex enum.
  std::vector<std::string> joint_names_;

  // ── UART / micro-ROS connection ────────────────────────────────────────

  /// Serial port device (from URDF <param name="serial_port">).
  std::string serial_port_;

  /// Baud rate (default: 921600).
  int baud_rate_;

  /// File descriptor for the serial port (populated in on_configure).
  int serial_fd_{-1};

  // ── Logger ────────────────────────────────────────────────────────────
  rclcpp::Logger logger_{rclcpp::get_logger("MeerSystemInterface")};
};

}  // namespace meer_hardware

#endif  // MEER_HARDWARE__MEER_SYSTEM_INTERFACE_HPP_
