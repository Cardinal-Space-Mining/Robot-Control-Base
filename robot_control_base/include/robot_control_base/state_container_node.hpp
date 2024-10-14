#pragma once

#include <geometry_msgs/msg/pose.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "custom_types/msg/talon_ctrl.hpp"
#include "custom_types/msg/talon_info.hpp"
#include "robot_control_base/base_state.hpp"

// Holds the common inputs a robot control needs
// When a motor state changes, gamepad input is given, or location changes,
// the change is forwarded on to the current state the state machine is in
class RobotContainer : public rclcpp::Node
{
public:
    RobotContainer(std::unique_ptr<BaseState> start_state, std::string name,
                   rclcpp::NodeOptions options);
    inline RobotContainer(std::unique_ptr<BaseState> start_state,
                          std::string name)
    : RobotContainer(std::move(start_state), name, rclcpp::NodeOptions{})
    {
    }
    RobotContainer(RobotContainer &&) = delete;
    RobotContainer & operator=(RobotContainer &&) = delete;

private:
    // Applies robot control message to Robot
    void apply_ctrl(const robot_control & ctrl);

    // Updates Base State
    void update();

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
    talon_ctrl_pub(const std::string & name);

    void localization_msg_cb();

private:
    std::unique_ptr<BaseState> current_state;
    robot_info robot_state;

private:
    // TODO Add current pose updating

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_sub;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        right_track_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        right_track_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        left_track_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        left_track_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        trencher_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        trencher_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        hopper_belt_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        hopper_belt_info;

    std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
        hopper_actuator_ctrl;
    std::shared_ptr<rclcpp::Subscription<custom_types::msg::TalonInfo>>
        hopper_actuator_info;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose>> traversal_dst;

private: // Localization Stuff
    const std::string robot_frame = "base_link";
    const std::string world_frame = "world";
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};