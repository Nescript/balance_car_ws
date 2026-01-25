#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <filters/realtime_circular_buffer.h>

namespace balance_controller {

class BalanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>{
public:
  BalanceController() = default;
  ~BalanceController() = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  hardware_interface::JointHandle left_wheel_joint_, right_wheel_joint_;
  ros::Subscriber imu_sub_;
  ros::Subscriber cmd_vel_sub_;
  
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

  control_toolbox::Pid balance_pid_;
  // control_toolbox::Pid left_velocity_pid_, right_velocity_pid_;
  
  control_toolbox::Pid velocity_pid_;

  
  double balance_cmd_ = 0.0;

  // double velocity_cmd_left_ = 0.0;
  // double velocity_cmd_right_ = 0.0;
  double velocity_cmd_ = 0.0;
  
  double target_linear_vel_ = 0.0;
  double target_angular_vel_ = 0.0;
  
  double current_pitch_ = 0.0;
  double target_pitch_ = 0.0;
  
  ros::Time last_imu_time_;

  double wheel_separation_ = 0.53; // 轮距
  double wheel_radius_ = 0.1;      // 轮子半径

  filters::RealtimeCircularBuffer<double> velocity_buffer_{10, 0.0};
  double filtered_linear_vel_ = 0.0;
  ros::Time last_info_time_;
};

} // namespace balance_controller