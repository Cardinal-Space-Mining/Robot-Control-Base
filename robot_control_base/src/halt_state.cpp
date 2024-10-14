#include "robot_control_base/halt_state.hpp"

std::optional<std::unique_ptr<BaseState>> HaltState::next()
{
    return std::nullopt;
};

robot_control HaltState::update(const robot_info &)
{
    robot_control ctrl{};
    ctrl.hopper_actuator.emplace(motor_control::ControlType::DISABLED, 0.0);
    ctrl.hopper_belt.emplace(motor_control::ControlType::DISABLED, 0.0);
    ctrl.track_left.emplace(motor_control::ControlType::DISABLED, 0.0);
    ctrl.track_right.emplace(motor_control::ControlType::DISABLED, 0.0);
    ctrl.trencher.emplace(motor_control::ControlType::DISABLED, 0.0);
    return ctrl;
}
