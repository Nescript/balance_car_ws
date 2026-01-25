#include "balance_controller/balance_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
  
  if (!balance_pid_.init(ros::NodeHandle(controller_nh, "balance_pid"))) {
    ROS_ERROR("Failed to initialize balance PID");
    return false;
  }
  if (!velocity_pid_.init(ros::NodeHandle(controller_nh, "velocity_pid"))) {
    ROS_ERROR("Failed to initialize left velocity PID");
    return false;
  }

  controller_nh.param("wheel_separation", wheel_separation_, 0.2);
  controller_nh.param("wheel_radius", wheel_radius_, 0.03);

  imu_sub_ = root_nh.subscribe("imu", 1, &BalanceController::imuCallback, this);
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &BalanceController::cmdVelCallback, this);

  return true;
}

void BalanceController::starting(const ros::Time& time) {
  balance_pid_.reset();
  velocity_pid_.reset();

  target_linear_vel_ = 0.0;
  target_angular_vel_ = 0.0;
  target_pitch_ = 0.0;
  
  velocity_buffer_.clear();
  filtered_linear_vel_ = 0.0;
  last_info_time_ = time;
}

void BalanceController::update(const ros::Time& time, const ros::Duration& period) {
  // 1. 获取当前状态反馈 (Forward 为正)
// 1. 获取原始线速度 (从 effort_joint_interface 获取)
    double raw_vel = (left_wheel_joint_.getVelocity() + right_wheel_joint_.getVelocity()) / 2.0 * wheel_radius_;

    // 2. 将新数据推入缓冲区
    velocity_buffer_.push_back(raw_vel);

    // 3. 计算缓冲区内所有数据的平均值
    double sum = 0.0;
    for (size_t i = 0; i < velocity_buffer_.size(); ++i) {
        sum += velocity_buffer_[i];
    }
    
    if (velocity_buffer_.size() > 0) {
        filtered_linear_vel_ = sum / velocity_buffer_.size();
    }
  // 如果期望前进，速度环会输出一个负的目标俯仰角（让车前倾）
  // double v_error = target_linear_vel_ - filtered_linear_vel_;
  double v_error = target_linear_vel_ - raw_vel;

  target_pitch_ = velocity_pid_.computeCommand(v_error, period);

  double pitch_error = current_pitch_ - target_pitch_;

  double base_effort = balance_pid_.computeCommand(pitch_error, period);

  double left_total_cmd = base_effort;
  double right_total_cmd = base_effort;

  left_wheel_joint_.setCommand(left_total_cmd);
  right_wheel_joint_.setCommand(right_total_cmd);
  if (time - last_info_time_ > ros::Duration(1.5)) {
    last_info_time_ = time;
    ROS_INFO("Pitch: %.4f, Target Pitch: %.4f, Base Effort: %.4f, Current Vel: %.4f, Target Vel: %.4f, Left Cmd: %.4f, Right Cmd: %.4f",
           current_pitch_, target_pitch_, base_effort, filtered_linear_vel_, target_linear_vel_, left_total_cmd, right_total_cmd);
  }
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
}

void BalanceController::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
  target_linear_vel_ = msg->linear.x;
  target_angular_vel_ = msg->angular.z;
}

} // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerBase)