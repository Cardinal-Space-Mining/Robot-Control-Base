#include "robot_control_base/state_container_node.hpp"

void RobotContainer::apply_ctrl(const robot_control &ctrl)
{
    if (ctrl.destination.has_value())
    {
        this->traversal_dst->publish(geometry_msgs::msg::Pose(ctrl.destination.value()));
    }

    if (ctrl.hopper_actuator.has_value())
    {
        this->hopper_actuator_ctrl->publish(custom_types::msg::TalonCtrl(ctrl.hopper_actuator.value()));
    }

    if (ctrl.hopper_belt.has_value())
    {
        this->hopper_belt_ctrl->publish(custom_types::msg::TalonCtrl(ctrl.hopper_belt.value()));
    }

    if (ctrl.track_left.has_value())
    {
        this->left_track_ctrl->publish(custom_types::msg::TalonCtrl(ctrl.track_left.value()));
    }

    if (ctrl.track_right.has_value())
    {
        this->right_track_ctrl->publish(custom_types::msg::TalonCtrl(ctrl.track_right.value()));
    }

    if (ctrl.trencher.has_value())
    {
        this->trencher_ctrl->publish(custom_types::msg::TalonCtrl(ctrl.trencher.value()));
    }
}

void RobotContainer::update()
{
    auto new_bot_ctrl = current_state->update(this->robot_state);

    apply_ctrl(new_bot_ctrl);

    auto new_state = current_state->next();
    if (new_state.has_value())
    {
        current_state = std::move(new_state.value());
    }
}

std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>> RobotContainer::talon_ctrl_pub(const std::string &name)
{
    return this->create_publisher<custom_types::msg::TalonCtrl>(name, 10);
}

RobotContainer::RobotContainer(std::unique_ptr<BaseState> start_state, std::string name, rclcpp::NodeOptions options) : rclcpp::Node(name, options),
                                                                                                                          current_state(std::move(start_state)),
                                                                                                                          joy_sub(this->create_subscription<sensor_msgs::msg::Joy>(
                                                                                                                              "joy", 10,
                                                                                                                              [this](const sensor_msgs::msg::Joy &joy)
                                                                                                                              { this->robot_state.joystick = joy; this->update(); }))

                                                                                                                          ,
                                                                                                                          right_track_ctrl(talon_ctrl_pub("track_right_ctrl")), right_track_info(this->create_subscription<custom_types::msg::TalonInfo>(
                                                                                                                                                                                    "track_right_info", 10, [this](const custom_types::msg::TalonInfo &msg)
                                                                                                                                                                                    { this->robot_state.track_right = msg; this->update(); })),
                                                                                                                          left_track_ctrl(talon_ctrl_pub("track_left_ctrl")), left_track_info(this->create_subscription<custom_types::msg::TalonInfo>(
                                                                                                                                                                                  "track_left_info", 10, [this](const custom_types::msg::TalonInfo &msg)
                                                                                                                                                                                  { this->robot_state.track_left = msg; this->update(); })),
                                                                                                                          trencher_ctrl(talon_ctrl_pub("trencher_ctrl")), trencher_info(this->create_subscription<custom_types::msg::TalonInfo>(
                                                                                                                                                                              "trencher_info", 10, [this](const custom_types::msg::TalonInfo &msg)
                                                                                                                                                                              { this->robot_state.trencher = msg; this->update(); }))

                                                                                                                          ,
                                                                                                                          hopper_belt_ctrl(talon_ctrl_pub("hopper_belt_ctrl")), hopper_belt_info(this->create_subscription<custom_types::msg::TalonInfo>(
                                                                                                                                                                                    "hopper_belt_info", 10, [this](const custom_types::msg::TalonInfo &msg)
                                                                                                                                                                                    { this->robot_state.hopper_belt = msg; this->update(); }))

                                                                                                                          ,
                                                                                                                          hopper_actuator_ctrl(talon_ctrl_pub("hopper_actuator_ctrl")), hopper_actuator_info(this->create_subscription<custom_types::msg::TalonInfo>(
                                                                                                                                                                                            "hopper_actuator_info", 10,
                                                                                                                                                                                            [this](const custom_types::msg::TalonInfo &msg)
                                                                                                                                                                                            { this->robot_state.hopper_actuator = msg; this->update(); })),
                                                                                                                          traversal_dst(this->create_publisher<geometry_msgs::msg::Pose>("traversal", 10)),
                                                                                                                          timer_(this->create_wall_timer(
                                                                                                                              std::chrono::milliseconds(100),
                                                                                                                              [this]() {}))
{
    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void RobotContainer::localization_msg_cb()
{

    geometry_msgs::msg::TransformStamped transform;

    try
    {
        // Look up the transform from "world" to "base_link"
        transform = tf2_buffer_->lookupTransform(this->robot_frame, this->world_frame, tf2::TimePointZero);

        // Convert TransformStamped to PoseStamped
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = transform.header.stamp;
        pose_stamped.header.frame_id = transform.header.frame_id;
        pose_stamped.pose.position.x = transform.transform.translation.x;
        pose_stamped.pose.position.y = transform.transform.translation.y;
        pose_stamped.pose.position.z = transform.transform.translation.z;
        pose_stamped.pose.orientation = transform.transform.rotation;

        this->robot_state.current_pose = pose_stamped.pose;

        // Log the pose data for debugging
        RCLCPP_INFO(this->get_logger(), "Pose2d - x: %f, y: %f, theta: %f",
                    this->robot_state.current_pose.x,
                    this->robot_state.current_pose.y,
                    this->robot_state.current_pose.theta);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(
            // Print exception which was caught
            this->get_logger(), "Failure %s\n", ex.what());
    }
    this->update();
}
