#pragma once

#include <optional>
#include <cstdint>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"

struct motor_info
{
    double temperature;
    double bus_voltage;

    double output_percent;
    double output_voltage;
    double output_current;

    double position;
    double velocity;

    motor_info() = default;
    motor_info(const custom_types::msg::TalonInfo &info) : temperature(info.temperature),
                                                           bus_voltage(info.bus_voltage),
                                                           output_percent(info.output_percent),
                                                           output_voltage(info.output_voltage),
                                                           position(info.position),
                                                           velocity(info.velocity)
    {
    }
};

struct motor_control
{
    enum class ControlType : int8_t
    {
        PERCENT_OUTPUT = 0,
        POSITION = 1,
        VELOCITY = 2,
        CURRENT = 3,
        FOLLOWER = 5,
        MOTION_PROFILE = 6,
        MOTION_MAGIC = 7,
        MOTION_PROFILE_ARC = 10,
        MUSIC_TONE = 13,
        DISABLED = 15

    };

    operator custom_types::msg::TalonCtrl() const
    {
        custom_types::msg::TalonCtrl msg;
        msg.mode = static_cast<int8_t>(this->mode);
        msg.value = this->value;
        return msg;
    }

    int8_t mode;
    double value;
};

struct pose2d
{
    double x, y, theta;
    pose2d() = default;
    pose2d(const geometry_msgs::msg::Pose &pose) : x(pose.position.x),
                                                   y(pose.position.y),
                                                   theta(theta_from_quanternion(pose.orientation))
    {
    }
    static double theta_from_quanternion(const geometry_msgs::msg::Quaternion &qt)
    {
        tf2::Quaternion quat_tf;
        tf2::convert(qt, quat_tf);
        tf2::Matrix3x3 m(quat_tf);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // This is not in game format, it is in mathematical format.
    static geometry_msgs::msg::Quaternion quanternion_from_rpy(double roll, double pitch, double yaw) // roll (x), pitch (y), yaw (z), angles are in radians
    {
        // Abbreviations for the various angular functions

        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        geometry_msgs::msg::Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    operator geometry_msgs::msg::Pose() const
    {
        geometry_msgs::msg::Pose msg;
        msg.position.x = this->x;
        msg.position.y = this->y;
        msg.orientation = quanternion_from_rpy(0, 0, this->theta);
        return msg;
    }
};

struct robot_info
{
    sensor_msgs::msg::Joy joystick;
    motor_info track_right;
    motor_info track_left;
    motor_info hopper_actuator;
    motor_info hopper_belt;
    motor_info trencher;
    double hopper_fullness;
    pose2d current_pose;
};

struct robot_control
{
    std::optional<motor_control> track_right;
    std::optional<motor_control> track_left;
    std::optional<motor_control> hopper_actuator;
    std::optional<motor_control> hopper_belt;
    std::optional<motor_control> trencher;
    std::optional<pose2d> destination;
};
