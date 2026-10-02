#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * @file mission_control_node.cpp
 * @brief Boilerplate mission control node for the MEER rover.
 *
 * Implements a top-level finite state machine (FSM) that sequences
 * all competition tasks for NAGC 2026.
 *
 * States:
 *   IDLE → LOCALIZING → NAVIGATING → TASK_EXECUTING → RETURNING → DONE
 *
 * TODO:
 *   - Integrate BehaviorTree.CPP or nav2_behavior_tree for robust FSM.
 *   - Subscribe to /obstacle_detected for reactive avoidance.
 *   - Load mission waypoints from YAML on startup.
 *   - Publish /mission_status [std_msgs/String] for operator HMI.
 *   - Implement operator E-STOP override via /e_stop topic.
 */

enum class MissionState
{
  IDLE,
  LOCALIZING,
  NAVIGATING,
  TASK_EXECUTING,
  RETURNING,
  DONE,
  FAULT,
};

const char * state_to_str(MissionState s)
{
  switch (s) {
    case MissionState::IDLE:           return "IDLE";
    case MissionState::LOCALIZING:     return "LOCALIZING";
    case MissionState::NAVIGATING:     return "NAVIGATING";
    case MissionState::TASK_EXECUTING: return "TASK_EXECUTING";
    case MissionState::RETURNING:      return "RETURNING";
    case MissionState::DONE:           return "DONE";
    case MissionState::FAULT:          return "FAULT";
    default:                           return "UNKNOWN";
  }
}

class MissionControlNode : public rclcpp::Node
{
public:
  MissionControlNode()
  : Node("mission_control_node"),
    state_(MissionState::IDLE)
  {
    // Parameters
    this->declare_parameter("auto_start", false);
    this->declare_parameter("mission_file", "");

    auto_start_    = this->get_parameter("auto_start").as_bool();
    mission_file_  = this->get_parameter("mission_file").as_string();

    // Publishers
    pub_status_ = this->create_publisher<std_msgs::msg::String>(
      "/mission_status", rclcpp::QoS(10).transient_local());

    // Subscribers
    sub_estop_ = this->create_subscription<std_msgs::msg::String>(
      "/e_stop",
      rclcpp::QoS(1).best_effort(),
      [this](const std_msgs::msg::String::SharedPtr /*msg*/) {
        RCLCPP_ERROR(this->get_logger(), "E-STOP received! Transitioning to FAULT.");
        transition_to(MissionState::FAULT);
      });

    // State machine timer — runs at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MissionControlNode::state_machine_tick, this));

    publish_status();
    RCLCPP_INFO(this->get_logger(),
      "MissionControlNode initialised. auto_start=%s",
      auto_start_ ? "true" : "false");

    if (auto_start_) {
      transition_to(MissionState::LOCALIZING);
    }
  }

private:
  void state_machine_tick()
  {
    switch (state_) {
      case MissionState::IDLE:
        // Wait for operator START command (to be implemented)
        break;

      case MissionState::LOCALIZING:
        // TODO: Wait for /initialpose confirmation from AMCL/EKF
        // Transition to NAVIGATING when localisation confidence > threshold
        RCLCPP_INFO_ONCE(this->get_logger(), "Localizing... (stub)");
        break;

      case MissionState::NAVIGATING:
        // TODO: Send next waypoint to WaypointNavigator
        // Transition to TASK_EXECUTING when within task zone radius
        RCLCPP_INFO_ONCE(this->get_logger(), "Navigating... (stub)");
        break;

      case MissionState::TASK_EXECUTING:
        // TODO: Execute competition task (artefact pickup, gate traversal, etc.)
        RCLCPP_INFO_ONCE(this->get_logger(), "Task executing... (stub)");
        break;

      case MissionState::RETURNING:
        // TODO: Navigate back to start zone
        RCLCPP_INFO_ONCE(this->get_logger(), "Returning... (stub)");
        break;

      case MissionState::DONE:
        RCLCPP_INFO_ONCE(this->get_logger(), "Mission COMPLETE.");
        break;

      case MissionState::FAULT:
        RCLCPP_ERROR_ONCE(this->get_logger(), "FAULT state — manual intervention required.");
        break;
    }
  }

  void transition_to(MissionState new_state)
  {
    RCLCPP_INFO(this->get_logger(),
      "Mission state: %s → %s",
      state_to_str(state_), state_to_str(new_state));
    state_ = new_state;
    publish_status();
  }

  void publish_status()
  {
    auto msg = std_msgs::msg::String();
    msg.data = state_to_str(state_);
    pub_status_->publish(msg);
  }

  // ── Members ─────────────────────────────────────────────────────────────
  MissionState state_;
  bool auto_start_;
  std::string mission_file_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_estop_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControlNode>());
  rclcpp::shutdown();
  return 0;
}
