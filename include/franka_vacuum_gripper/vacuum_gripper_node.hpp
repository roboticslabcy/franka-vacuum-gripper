#pragma once

#include <memory>
#include <string>
#include <thread>

#include <franka/vacuum_gripper.h>
#include <franka/vacuum_gripper_state.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <franka_vacuum_gripper/msg/vacuum_gripper_state.hpp>
#include <franka_vacuum_gripper/srv/drop_off.hpp>
#include <franka_vacuum_gripper/srv/vacuum.hpp>

namespace franka_vacuum_gripper {

class VacuumGripperNode : public rclcpp::Node {
 public:
  explicit VacuumGripperNode(const rclcpp::NodeOptions& options);
  ~VacuumGripperNode() override;

 private:
  // libfranka vacuum gripper instance
  std::unique_ptr<franka::VacuumGripper> gripper_;

  // Publishers
  rclcpp::Publisher<msg::VacuumGripperState>::SharedPtr state_publisher_;

  // Services
  rclcpp::Service<srv::Vacuum>::SharedPtr vacuum_service_;
  rclcpp::Service<srv::DropOff>::SharedPtr drop_off_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  // State publishing timer
  rclcpp::TimerBase::SharedPtr state_timer_;

  // Service callbacks
  void handleVacuum(const std::shared_ptr<srv::Vacuum::Request> request,
                    std::shared_ptr<srv::Vacuum::Response> response);

  void handleDropOff(const std::shared_ptr<srv::DropOff::Request> request,
                     std::shared_ptr<srv::DropOff::Response> response);

  void handleStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publishState();

  static franka::VacuumGripper::ProductionSetupProfile profileFromString(
      const std::string& profile_str);

  static msg::VacuumGripperState toRosMsg(const franka::VacuumGripperState& state);
};

}  // namespace franka_vacuum_gripper
