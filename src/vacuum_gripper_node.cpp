#include "franka_vacuum_gripper/vacuum_gripper_node.hpp"

#include <chrono>
#include <stdexcept>

#include <franka/exception.h>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace franka_vacuum_gripper {

VacuumGripperNode::VacuumGripperNode(const rclcpp::NodeOptions& options)
    : Node("franka_vacuum_gripper", options) {
  // Declare parameters
  declare_parameter<std::string>("robot_ip", "");
  declare_parameter<double>("state_publish_rate", 30.0);

  const std::string robot_ip = get_parameter("robot_ip").as_string();
  if (robot_ip.empty()) {
    throw std::invalid_argument("Parameter 'robot_ip' must be set");
  }

  RCLCPP_INFO(get_logger(), "Connecting to vacuum gripper at %s ...", robot_ip.c_str());
  try {
    gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip);
    RCLCPP_INFO(get_logger(), "Connected. Server version: %d", gripper_->serverVersion());
  } catch (const franka::Exception& e) {
    throw std::runtime_error(std::string("Failed to connect to vacuum gripper: ") + e.what());
  }

  // Publisher
  state_publisher_ = create_publisher<msg::VacuumGripperState>("~/state", 10);

  // Services
  vacuum_service_ = create_service<srv::Vacuum>(
      "~/vacuum",
      [this](const std::shared_ptr<srv::Vacuum::Request> req,
             std::shared_ptr<srv::Vacuum::Response> res) { handleVacuum(req, res); });

  drop_off_service_ = create_service<srv::DropOff>(
      "~/drop_off",
      [this](const std::shared_ptr<srv::DropOff::Request> req,
             std::shared_ptr<srv::DropOff::Response> res) { handleDropOff(req, res); });

  stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) { handleStop(req, res); });

  // State publishing timer
  const double rate = get_parameter("state_publish_rate").as_double();
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate));
  state_timer_ = create_wall_timer(period, [this]() { publishState(); });

  RCLCPP_INFO(get_logger(), "Vacuum gripper node ready (state at %.1f Hz)", rate);
}

VacuumGripperNode::~VacuumGripperNode() = default;

void VacuumGripperNode::handleVacuum(const std::shared_ptr<srv::Vacuum::Request> request,
                                     std::shared_ptr<srv::Vacuum::Response> response) {
  const auto profile = profileFromString(request->profile);
  const auto timeout = std::chrono::milliseconds(request->timeout_ms);

  RCLCPP_INFO(get_logger(), "Vacuum: setpoint=%u mbar, timeout=%d ms, profile=%s",
              request->vacuum, request->timeout_ms, request->profile.c_str());

  try {
    response->success = gripper_->vacuum(request->vacuum, timeout, profile);
    response->message = response->success ? "Vacuum established" : "Vacuum not established within timeout";
  } catch (const franka::CommandException& e) {
    response->success = false;
    response->message = std::string("Command error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "Vacuum command failed: %s", e.what());
  } catch (const franka::NetworkException& e) {
    response->success = false;
    response->message = std::string("Network error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "Network error during vacuum: %s", e.what());
  }
}

void VacuumGripperNode::handleDropOff(const std::shared_ptr<srv::DropOff::Request> request,
                                      std::shared_ptr<srv::DropOff::Response> response) {
  const auto timeout = std::chrono::milliseconds(request->timeout_ms);

  RCLCPP_INFO(get_logger(), "DropOff: timeout=%d ms", request->timeout_ms);

  try {
    response->success = gripper_->dropOff(timeout);
    response->message = response->success ? "Drop-off successful" : "Drop-off failed";
  } catch (const franka::CommandException& e) {
    response->success = false;
    response->message = std::string("Command error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "Drop-off command failed: %s", e.what());
  } catch (const franka::NetworkException& e) {
    response->success = false;
    response->message = std::string("Network error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "Network error during drop-off: %s", e.what());
  }
}

void VacuumGripperNode::handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(get_logger(), "Stop requested");

  try {
    response->success = gripper_->stop();
    response->message = response->success ? "Stopped" : "Stop command returned false";
  } catch (const franka::CommandException& e) {
    response->success = false;
    response->message = std::string("Command error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "Stop command failed: %s", e.what());
  } catch (const franka::NetworkException& e) {
    response->success = false;
    response->message = std::string("Network error: ") + e.what();
    RCLCPP_ERROR(get_logger(), "Network error during stop: %s", e.what());
  }
}

void VacuumGripperNode::publishState() {
  try {
    const auto state = gripper_->readOnce();
    auto msg = toRosMsg(state);
    msg.header.stamp = now();
    state_publisher_->publish(msg);
  } catch (const franka::NetworkException& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Failed to read vacuum gripper state: %s", e.what());
  } catch (const franka::InvalidOperationException& e) {
    // Another readOnce is already running — skip this cycle silently
  }
}

franka::VacuumGripper::ProductionSetupProfile VacuumGripperNode::profileFromString(
    const std::string& profile_str) {
  if (profile_str == "P1") {
    return franka::VacuumGripper::ProductionSetupProfile::kP1;
  } else if (profile_str == "P2") {
    return franka::VacuumGripper::ProductionSetupProfile::kP2;
  } else if (profile_str == "P3") {
    return franka::VacuumGripper::ProductionSetupProfile::kP3;
  }
  // Default P0
  return franka::VacuumGripper::ProductionSetupProfile::kP0;
}

msg::VacuumGripperState VacuumGripperNode::toRosMsg(const franka::VacuumGripperState& state) {
  msg::VacuumGripperState ros_state;
  ros_state.in_control_range = state.in_control_range;
  ros_state.part_detached = state.part_detached;
  ros_state.part_present = state.part_present;
  ros_state.actual_power = state.actual_power;
  ros_state.vacuum = state.vacuum;

  switch (state.device_status) {
    case franka::VacuumGripperDeviceStatus::kGreen:
      ros_state.device_status = msg::VacuumGripperState::DEVICE_STATUS_GREEN;
      break;
    case franka::VacuumGripperDeviceStatus::kYellow:
      ros_state.device_status = msg::VacuumGripperState::DEVICE_STATUS_YELLOW;
      break;
    case franka::VacuumGripperDeviceStatus::kOrange:
      ros_state.device_status = msg::VacuumGripperState::DEVICE_STATUS_ORANGE;
      break;
    case franka::VacuumGripperDeviceStatus::kRed:
      ros_state.device_status = msg::VacuumGripperState::DEVICE_STATUS_RED;
      break;
  }

  return ros_state;
}

}  // namespace franka_vacuum_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(franka_vacuum_gripper::VacuumGripperNode)
