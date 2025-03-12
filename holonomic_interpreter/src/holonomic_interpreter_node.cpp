#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pid.h"

#include <limits>
#include <math.h>

class HolonomicInterpreter : public rclcpp::Node
{
  public:
    HolonomicInterpreter() : Node("holonomic_interpreter")
    {
      //Qos profile
      auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);

      // Subscribers
      cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("holo_cmd_vel", sub_qos,
        std::bind(&HolonomicInterpreter::callback_cmd, this, std::placeholders::_1));

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", sub_qos,
        std::bind(&HolonomicInterpreter::callback_odom, this, std::placeholders::_1));

      //imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("sensors/imu_0/data", sub_qos,
      //  std::bind(&HolonomicInterpreter::callback_imu, this, std::placeholders::_1));

      // Publishers
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("nonholo_cmd_vel",10);
      error_pub_ = this->create_publisher<std_msgs::msg::Float64>("control_error",10);
      desired_pub_ = this->create_publisher<std_msgs::msg::Float64>("control_setpoint",10);
      cmd_pub_stamped = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("nonholo_cmd_vel_stamped",10);
      rpy_pub_ = this->create_publisher<std_msgs::msg::Float64>("heading",10);
      
      //Desired state
      desired_heading_ = 0;
      desired_speed_ = 0;
      last_speed_ = 0.0;
      delta_speed_ = 0.05;

      //Current state
      roll_ = 0;
      pitch_ = 0;
      yaw_ = 0;
      roll_dot_ = 0;
      pitch_dot_ = 0;
      yaw_dot_ = 0;

      // PID controllers
      this->declare_parameter("heading_kp", 1.0);
      this->get_parameter("heading_kp",heading_kp_);

      this->declare_parameter("heading_ki", 0.00);
      this->get_parameter("heading_ki", heading_ki_);

      this->declare_parameter("heading_kd", 0.1);
      this->get_parameter("heading_kd", heading_kd_);

      this->declare_parameter("max_vel", 0.5);
      this->get_parameter("max_vel", max_vel_);

      this->declare_parameter("min_vel", 0.15);
      this->get_parameter("min_vel", min_vel_);

      this->declare_parameter("max_ang_acc", 0.05);
      this->get_parameter("max_ang_acc", delta_speed_);

      this->declare_parameter("origin_rotation", -0.6021965548550632);
      this->get_parameter("origin_rotation", origin_rotation_);

      double inf = std::numeric_limits<double>::infinity();

      //Update rate
      double rate = 0.05;
      //estop_ = false;

      heading_pid_ = new PID(rate, inf, -inf, heading_kp_, heading_kd_, heading_ki_);

      std::chrono::milliseconds rate_ms((int)(rate*1000));
      timer_ = this->create_wall_timer(rate_ms, std::bind(&HolonomicInterpreter::callback_timer, this));
    }

  private:
    void callback_timer()
    {
      desired_heading_ = std::atan2(lin_vel_y_, lin_vel_x_); 
      desired_speed_ = std::sqrt((lin_vel_x_*lin_vel_x_) + (lin_vel_y_*lin_vel_y_));

      double heading_error = wrap_angle(desired_heading_ - yaw_);
      double heading_correction = heading_kp_*heading_error - heading_kd_*yaw_dot_;    //heading_pid_->calculate(-heading_error, -yaw_dot_);

      //RCLCPP_WARN(this->get_logger(), "Heading: %f", yaw_);
      //RCLCPP_WARN(this->get_logger(), "Setpoint: %f", desired_heading_);

      heading_correction = last_speed_ + std::min(std::max(heading_correction - last_speed_, -delta_speed_), delta_speed_);
      //WRCLCPP_WARN(this->get_logger(), "Input: %f", heading_correction);

      if (safety_check(cmd_epoch_))
      {
        if (safety_check(odom_epoch_))
        {
          auto twist = geometry_msgs::msg::Twist();
          auto twist_stamped = geometry_msgs::msg::TwistWithCovarianceStamped();

          //if (desired_speed_ != 0.0)
          //{

          //double val = std::max(std::min(heading_correction, M_PI/24), -M_PI/24);
          //val = std::abs(val/(M_PI/24));
          speed_ = desired_speed_; //(desired_speed_-val*desired_speed_)+min_vel_;

          if (speed_ >= max_vel_) {
              speed_ = max_vel_;
          } else if (speed_ <= -max_vel_) {
              speed_ = -max_vel_;
          } //else {
              // No change needed, as speed_ remains the same
          //}

          twist.angular.z = heading_correction;
          twist.linear.x = speed_;

          twist_stamped.twist.twist.angular.z = heading_correction;
          twist_stamped.twist.twist.linear.x = speed_;
          twist_stamped.header.stamp = this->get_clock()->now();
          twist_stamped.header.frame_id = "base_link";

          // if (estop_)
          // {
          //   twist.linear.x = 0.0;
          //   twist.angular.z = 0.0;
          //   RCLCPP_WARN(this->get_logger(), "Estop engaged.");
          // }

          cmd_pub_->publish(twist);
          cmd_pub_stamped->publish(twist_stamped);
          last_speed_ = heading_correction;

          auto error = std_msgs::msg::Float64();
          error.data = heading_error;

          error_pub_->publish(error);

          auto desired = std_msgs::msg::Float64();
          desired.data = desired_heading_;

          desired_pub_->publish(desired);
        }
         else
         {
             RCLCPP_WARN(this->get_logger(), "MISSING: HEADING!");
         }
      }
       else
       {
           RCLCPP_WARN(this->get_logger(), "MISSING: SETPOINT!");
       }
    }

    void callback_cmd(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        cmd_epoch_ = now();

        //RCLCPP_WARN(this->get_logger(), "Got HEADING!");
        
        lin_vel_x_ = msg->linear.x;
        lin_vel_y_ = msg->linear.y;
    }

    void callback_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
      //RCLCPP_WARN(this->get_logger(), "Got IMU!");
      odom_epoch_ = now();
        
      yaw_ = quat_to_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      roll_dot_ = msg->twist.twist.angular.x;
      pitch_dot_ = msg->twist.twist.angular.y;
      yaw_dot_ = msg->twist.twist.angular.z;

      auto debug_msg = std_msgs::msg::Float64();
      debug_msg.data = yaw_;
      rpy_pub_->publish(debug_msg);
    }


    void callback_imu(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
    {
      //RCLCPP_WARN(this->get_logger(), "Got IMU!");
      //odom_epoch_ = now();

      //yaw_ = quat_to_yaw(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

      //desired_pub_->publish(desired);
      //roll_dot_ = msg->twist.twist.angular.x;
      //pitch_dot_ = msg->twist.twist.angular.y;
      //yaw_dot_ = msg->twist.twist.angular.z;

      // auto debug_msg = std_msgs::msg::Float64();
      // debug_msg.data = yaw_
      // rpy_pub_->publish(debug_msg);
    }


    double quat_to_yaw(double x, double y, double z, double w) {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return yaw;
    }

    bool safety_check(rclcpp::Time epoch)
    {
      if ((now().nanoseconds() - epoch.nanoseconds()) * 1e-9 > 1.0)
      {
        //RCLCPP_INFO(this->get_logger(), "Missing sensor update rate!");
        return false;
      }
      return true;
    }

    double wrap_angle (double angle)
    {
      angle = fmod(angle + M_PI, 2.0 * M_PI);

      if (angle < 0)
      {
        angle += 2.0*M_PI;
      }

      return angle - M_PI;
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;


    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr cmd_pub_stamped;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rpy_pub_;

    // ROS parameters
    double heading_kp_;
    double heading_ki_;
    double heading_kd_;
    double max_vel_;
    double min_vel_;
    double origin_rotation_;

    // Safety checks
    rclcpp::Time cmd_epoch_;
    rclcpp::Time odom_epoch_;

    PID *heading_pid_;

    //Desried state
    double lin_vel_x_;
    double lin_vel_y_;
    double last_speed_;
    double delta_speed_;

    double desired_heading_;
    double desired_speed_;
    double speed_;

    //Current state
    double roll_;
    double pitch_;
    double yaw_;
    double roll_dot_;
    double pitch_dot_;
    double yaw_dot_;
    bool estop_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto holo_interpret = std::make_shared<HolonomicInterpreter>();

  rclcpp::spin(holo_interpret);

  rclcpp::shutdown();

  return 0;
}
