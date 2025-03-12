#ifndef AVOIDANCE_HPP
#define AVOIDANCE_HPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <px4_safety_lib/PX4Safety.hpp>

#include <rclcpp/rclcpp.hpp>

class Avoidance : public rclcpp::Node {
    private:

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    geometry_msgs::msg::Twist unfiltered_vel_;
    geometry_msgs::msg::Twist filtered_vel_;
    geometry_msgs::msg::Pose agent_pose_;
    
    px4_safety_lib::PX4Safety px4_safety;




    void vel_callback(geometry_msgs::msg::Twist::SharedPtr msg);
    void pose_callback(nav_msgs::msg::Odometry::SharedPtr msg);

    public:
    Avoidance();
};



#endif
