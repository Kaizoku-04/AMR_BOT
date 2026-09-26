// Copyright 2025 ros2_control Development Team (joint_state_topic_hardware_interface 1.1.0), Apache-2.0.
// Modified 2026 for OpenAMR (see isaac_joint_system.hpp).
#include "open_amr_isaac_hardware/isaac_joint_system.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace open_amr_isaac_hardware
{
CallbackReturn IsaacJointSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  const auto param = [this](const std::string& name, const std::string& fallback) {
    const auto it = get_hardware_info().hardware_parameters.find(name);
    return it != get_hardware_info().hardware_parameters.end() ? it->second : fallback;
  };
  for (const auto& joint : get_hardware_info().joints)
  {
    joints_.push_back(joint.name);
    bool vel = false;
    for (const auto& ci : joint.command_interfaces)
    {
      vel = vel || ci.name == hardware_interface::HW_IF_VELOCITY;
    }
    has_velocity_command_.push_back(vel);
  }
  // commands: reliable, depth 1 (the latest is all that matters); states: sensor data QoS (the simulator publishes
  // them best effort every physics step)
  commands_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      param("joint_commands_topic", "/isaac_joint_commands"), rclcpp::QoS(1));
  states_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      param("joint_states_topic", "/isaac_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = *msg;
        have_state_ = true;
      });
  return CallbackReturn::SUCCESS;
}

CallbackReturn IsaacJointSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // start commanding where the arm is (ros2_control command interfaces are NaN until a controller writes them)
  read(get_node()->now(), rclcpp::Duration(0, 0));
  for (const auto& name : joints_)
  {
    set_command(name + "/" + hardware_interface::HW_IF_POSITION,
                get_state(name + "/" + hardware_interface::HW_IF_POSITION));
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type IsaacJointSystem::read(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/)
{
  sensor_msgs::msg::JointState s;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_state_)
    {
      return hardware_interface::return_type::OK;
    }
    s = latest_state_;
  }
  for (std::size_t i = 0; i < s.name.size(); ++i)
  {
    const std::string& n = s.name[i];
    if (i < s.position.size() && std::isfinite(s.position[i]) &&
        has_state(n + "/" + hardware_interface::HW_IF_POSITION))
    {
      set_state(n + "/" + hardware_interface::HW_IF_POSITION, s.position[i]);
    }
    if (i < s.velocity.size() && std::isfinite(s.velocity[i]) &&
        has_state(n + "/" + hardware_interface::HW_IF_VELOCITY))
    {
      set_state(n + "/" + hardware_interface::HW_IF_VELOCITY, s.velocity[i]);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacJointSystem::write(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  sensor_msgs::msg::JointState cmd;
  // the controller sampled its trajectory at `time`: the simulator extrapolates from this stamp to its physics step.
  // (The upstream interface stamps with its own node's clock, which lagged the controller's by one sim step now and
  // then: the arm then jumped ahead by a step, ~16 mm at 1 m/s — OpenAMR tracking test, 2026-09-26.)
  cmd.header.stamp = time;
  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    const double p = get_command(joints_[i] + "/" + hardware_interface::HW_IF_POSITION);
    const double v =
        has_velocity_command_[i] ? get_command(joints_[i] + "/" + hardware_interface::HW_IF_VELOCITY) : 0.0;
    if (!std::isfinite(p))
    {
      ++skipped_non_finite_;
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "non-finite position command for %s: not sent (%zu so far)", joints_[i].c_str(),
                           skipped_non_finite_);
      return hardware_interface::return_type::OK;
    }
    cmd.name.push_back(joints_[i]);
    cmd.position.push_back(p);
    cmd.velocity.push_back(std::isfinite(v) ? v : 0.0);
  }
  commands_pub_->publish(cmd);
  return hardware_interface::return_type::OK;
}
}  // namespace open_amr_isaac_hardware

PLUGINLIB_EXPORT_CLASS(open_amr_isaac_hardware::IsaacJointSystem, hardware_interface::SystemInterface)
