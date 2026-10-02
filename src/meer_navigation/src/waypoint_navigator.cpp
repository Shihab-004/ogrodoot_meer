#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @file waypoint_navigator.cpp
 * @brief Boilerplate waypoint navigation node for the MEER rover.
 *
 * Reads a list of GPS waypoints (or Cartesian poses in the map frame),
 * converts them, and sends NavigateToPose action goals to Nav2.
 *
 * TODO:
 *   - Load waypoints from a YAML config file.
 *   - Implement GPS → UTM → map-frame conversion using robot_localization.
 *   - Handle action feedback and result callbacks.
 *   - Integrate with meer_mission_control state machine via service/action.
 */

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class WaypointNavigator : public rclcpp::Node
{
public:
  WaypointNavigator()
  : Node("waypoint_navigator")
  {
    this->declare_parameter("frame_id", "map");
    frame_id_ = this->get_parameter("frame_id").as_string();

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(),
      "WaypointNavigator ready. Waiting for Nav2 action server...");
  }

  void navigate_to(double x, double y, double yaw = 0.0)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available!");
      return;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = frame_id_;
    goal.pose.header.stamp    = this->get_clock()->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    // Simple yaw-only quaternion: w=cos(yaw/2), z=sin(yaw/2)
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(this->get_logger(),
      "Sending goal: (%.2f, %.2f, yaw=%.2f)", x, y, yaw);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Waypoint reached!");
        } else {
          RCLCPP_WARN(this->get_logger(), "Navigation failed (code %d).",
            static_cast<int>(result.code));
        }
      };

    nav_client_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
