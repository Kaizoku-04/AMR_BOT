// Copyright 2025 ros2_control Development Team (joint_state_topic_hardware_interface 1.1.0), Apache-2.0.
// Modified 2026 for OpenAMR: sample-time stamps, finite-only commands, locked state hand-over, commands start at the
// measured state.
#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace open_amr_isaac_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class IsaacJointSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commands_pub_;
  std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_state_;   // guarded by state_mutex_
  bool have_state_{ false };                    // guarded by state_mutex_
  std::vector<std::string> joints_;
  std::vector<bool> has_velocity_command_;
  std::size_t skipped_non_finite_{ 0 };
};
}  // namespace open_amr_isaac_hardware
