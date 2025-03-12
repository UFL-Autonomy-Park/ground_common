#include "Avoidance.hpp"

using std::placeholders::_1;
Avoidance::Avoidance() : Node("avoidance_node") {

    // qos stuff
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    px4_safety.initialize(this);

    // subscribers
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_unfiltered", sub_qos, std::bind(&Avoidance::vel_callback, this, _1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/global", sub_qos, std::bind(&Avoidance::pose_callback, this, _1));

    // publishers
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_filtered", 10);
}

void Avoidance::pose_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    agent_pose_ = (*msg).pose.pose;
}

void Avoidance::vel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    unfiltered_vel_ = *msg;
    // Safety
    filtered_vel_ = px4_safety.compute_safe_cmd_vel(agent_pose_, unfiltered_vel_);
    filtered_vel_.linear.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "pose: x: %.3f, y: %.3f", agent_pose_.position.x, agent_pose_.position.y);
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = agent_pose_.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double r{}, p{}, y{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(r, p, y);
    geometry_msgs::msg::Twist tmp = filtered_vel_;
    //RCLCPP_INFO(this->get_logger(), "filtered x: %.3f, filtered y: %.3f", filtered_vel_.linear.x, filtered_vel_.linear.y);
    //filtered_vel_.linear.x = cos(y)*tmp.linear.x + sin(y)*tmp.linear.y;
    //filtered_vel_.linear.y = -sin(y)*tmp.linear.x + cos(y)*tmp.linear.y;

    RCLCPP_INFO(this->get_logger(), "filtered x: %.3f, filtered y: %.3f", filtered_vel_.linear.x, filtered_vel_.linear.y);


    vel_pub_->publish(filtered_vel_);
}
