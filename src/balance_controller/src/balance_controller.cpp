#include "balance_controller/balance_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <boost/concept_check.hpp>

namespace balance_controller {

bool BalanceController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  try {
    left_wheel_joint_ = effort_joint_interface->getHandle("left_wheel_joint");
    right_wheel_joint_ = effort_joint_interface->getHandle("right_wheel_joint");
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR("Could not get joint handles: %s", e.what());
    return false;
  }
  if (!yaw_pid_.init(ros::NodeHandle(controller_nh, "yaw_pid"))) {
    ROS_ERROR("Failed to initialize yaw PID");
    return false;
  }
  if (!controller_nh.getParam("k1", k1)) {
    ROS_WARN("Parameter 'k1' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k2", k2)) {
    ROS_WARN("Parameter 'k2' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k3", k3)) {
    ROS_WARN("Parameter 'k3' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k4", k4)) {
    ROS_WARN("Parameter 'k4' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k1_selfup", k1_selfup)) {
    ROS_WARN("Parameter 'k1_selfup' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k2_selfup", k2_selfup)) {
    ROS_WARN("Parameter 'k2_selfup' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k3_selfup", k3_selfup)) {
    ROS_WARN("Parameter 'k3_selfup' not set. Using default: %f", 0.0);
  }
  if (!controller_nh.getParam("k4_selfup", k4_selfup)) {
    ROS_WARN("Parameter 'k4_selfup' not set. Using default: %f", 0.0);
  }

  controller_nh.param("wheel_separation", wheel_separation_, 0.2);
  controller_nh.param("wheel_radius", wheel_radius_, 0.03);
  pos_error_pub_ = root_nh.advertise<std_msgs::Float64>("position_error", 1);
  vel_error_pub_ = root_nh.advertise<std_msgs::Float64>("velocity_error", 1);
  pitch_error_pub_ = root_nh.advertise<std_msgs::Float64>("pitch_error", 1);
  omega_error_pub_ = root_nh.advertise<std_msgs::Float64>("omega_error", 1);
  last_effort_pub_ = root_nh.advertise<std_msgs::Float64>("last_effort", 1);

  imu_sub_ = root_nh.subscribe("imu", 1, &BalanceController::imuCallback, this);
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &BalanceController::cmdVelCallback, this);

  return true;
}

void BalanceController::starting(const ros::Time& time) {
  target_linear_vel_ = 0.0;
  target_angular_vel_ = 0.0;
  target_pitch_ = 0.0;

  current_pos_ = 0.0;
  target_pos_ = 0.0;
  current_state_ = STATE_NORMAL;
}

void BalanceController::update(const ros::Time& time, const ros::Duration& period) {
  double dt = period.toSec();
  
  double current_raw_linear_vel_ = (left_wheel_joint_.getVelocity() + right_wheel_joint_.getVelocity()) * wheel_radius_ / 2.0;
  current_linear_vel_ = vel_kf.update(current_raw_linear_vel_, dt, last_effort_);


  // current_pos_ += current_linear_vel_ * dt;
  current_pos_ =(left_wheel_joint_.getPosition() + right_wheel_joint_.getPosition()) * wheel_radius_ / 2;
  target_pos_ += target_linear_vel_ * dt;
  
  double pos_error = current_pos_ - target_pos_; 
  double vel_error = current_linear_vel_ - target_linear_vel_;

  double pitch_error = current_pitch_ - target_pitch_;
  double omega_error = current_omega_ - target_omega_;
  
  // 倒地状态机
  switch (current_state_) {
    case STATE_NORMAL:
        if (std::abs(pitch_error) > 0.835) {
            current_state_ = STATE_FALLEN;
            ROS_INFO("Switched to FALLEN state");
        }
        break;

    case STATE_FALLEN:
        if (std::abs(omega_error) < 0.2 && std::abs(vel_error) < 0.01) {
            current_state_ = STATE_SELF_UP;
            self_up_start_time_ = time;
            last_effort_ = 0.0;
            ROS_INFO("Switched to SELF_UP state");
        }
        break;

    case STATE_SELF_UP:
        if ((time - self_up_start_time_).toSec() > 5.0) {
            current_state_ = STATE_NORMAL;
            ROS_INFO("Switched to NORMAL state");
        }
        break;
  }

  double base_effort = 0;
  if (current_state_ == STATE_SELF_UP) {
    base_effort = -(k1_selfup * pos_error + k2_selfup * vel_error + k3_selfup * pitch_error + k4_selfup * omega_error);
  }
  else if (current_state_ == STATE_NORMAL) {
    base_effort = -(k1 * pos_error + k2 * vel_error + k3 * pitch_error + k4 * omega_error);
  }
  else {
      base_effort = 0;
  }
  
  double yaw_effort = yaw_pid_.computeCommand(target_angular_vel_ - current_angular_vel_, period);
  
  left_wheel_joint_.setCommand(base_effort / 2 - yaw_effort);
  right_wheel_joint_.setCommand(base_effort / 2 + yaw_effort);
  // 倒地时候可以清空这个转向pid的输出
  last_effort_ = base_effort;

  // 发布误差信息
  std_msgs::Float64 pos_error_msg;
  pos_error_msg.data = pos_error;
  pos_error_pub_.publish(pos_error_msg);
  std_msgs::Float64 vel_error_msg;
  vel_error_msg.data = vel_error;
  vel_error_pub_.publish(vel_error_msg);
  std_msgs::Float64 pitch_error_msg;
  pitch_error_msg.data = pitch_error;
  pitch_error_pub_.publish(pitch_error_msg);
  std_msgs::Float64 omega_error_msg;
  omega_error_msg.data = omega_error;
  omega_error_pub_.publish(omega_error_msg);
  std_msgs::Float64 last_effort_msg;
  last_effort_msg.data = last_effort_;
  last_effort_pub_.publish(last_effort_msg);
}

void BalanceController::stopping(const ros::Time& time) {
  left_wheel_joint_.setCommand(0.0);
  right_wheel_joint_.setCommand(0.0);
}

void BalanceController::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_pitch_ = pitch;
  current_omega_ = msg->angular_velocity.y;
  current_angular_vel_ = msg->angular_velocity.z;
}

void BalanceController::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
  target_linear_vel_ = msg->linear.x;
  target_angular_vel_ = msg->angular.z;
}

} // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerBase)