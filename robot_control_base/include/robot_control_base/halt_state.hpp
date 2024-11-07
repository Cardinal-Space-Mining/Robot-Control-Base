#pragma once
#include "robot_control_base/base_state.hpp"

// This class just does nothing forever
class HaltState : public BaseState
{
public:
    HaltState() = default;
    ~HaltState() override = default;

public:
    // Returns what state the robot should go into next
    // If the optional is empty, we keep in the same state for the next system update
    std::optional<std::unique_ptr<BaseState>> next() override;

public:
    // Tells the robot to do nothing and sit there
    robot_control update(const robot_info &) override;
};
