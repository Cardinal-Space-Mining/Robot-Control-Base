#pragma once
#include <memory>
#include <optional>

#include "robot_control_base/robot_ctrl_types.hpp"

class BaseState
{
public:
    BaseState() = default;
    virtual ~BaseState() = default;

public:
    // Returns what state the robot should go into next
    // If the optional is empty, we keep in the same state for the next system update
    virtual std::optional<std::unique_ptr<BaseState>> next() = 0;

public:
    // Returns a robot_control struct to instruct the robot on what to do
    virtual robot_control update(const robot_info &info) = 0;
};