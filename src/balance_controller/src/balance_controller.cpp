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
  if (!position_pid_.init(ros::NodeHandle(controller_nh, "position_pid"))) {
    ROS_ERROR("Failed to initialize position PID");
    return false;
  }
  if (!yaw_pid_.init(ros::NodeHandle(controller_nh, "yaw_pid"))) {
    ROS_ERROR("Failed to initialize yaw PID");
    return false;
  }


  controller_nh.param("wheel_separation", wheel_separation_, 0.2);
  controller_nh.param("wheel_radius", wheel_radius_, 0.03);

  imu_sub_ = root_nh.subscribe("imu", 1, &BalanceController::imuCallback, this);
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &BalanceController::cmdVelCallback, this);
  combine_odom_sub_ = root_nh.subscribe("/robot_pose_ekf/combine_odom", 1, &BalanceController::cmdVelCallback, this); // --- IGNORE ---
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 50);
  x_ = 0.0; y_ = 0.0; theta_ = 0.0;

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
  double raw_vel = (left_wheel_joint_.getVelocity() + right_wheel_joint_.getVelocity()) / 2.0 * wheel_radius_;
  
  // 1. 俯仰和线速度控制（原有逻辑）
  double v_error = target_linear_vel_ - raw_vel;
  target_pitch_ = velocity_pid_.computeCommand(v_error, period);
  double pitch_error = current_pitch_ - target_pitch_;
  double base_effort = balance_pid_.computeCommand(pitch_error, period);


  double yaw_error = target_angular_vel_ - current_angular_vel_;
  double steering_effort = yaw_pid_.computeCommand(yaw_error, period);

  double left_total_cmd = base_effort - steering_effort;
  double right_total_cmd = base_effort + steering_effort;

  left_wheel_joint_.setCommand(left_total_cmd);
  right_wheel_joint_.setCommand(right_total_cmd);
  if (time - last_info_time_ > ros::Duration(1.5)) {
    last_info_time_ = time;
    ROS_INFO("Pitch: %.4f, Target Pitch: %.4f, Base Effort: %.4f, Current Vel: %.4f, Target Vel: %.4f, Left Cmd: %.4f, Right Cmd: %.4f",
           current_pitch_, target_pitch_, base_effort, filtered_linear_vel_, target_linear_vel_, left_total_cmd, right_total_cmd);
  }

  double dt = period.toSec();
  double v_left = left_wheel_joint_.getVelocity() * wheel_radius_;
  double v_right = right_wheel_joint_.getVelocity() * wheel_radius_;

  // 1. 运动学计算 (差分模型)
  double v_linear = (v_right + v_left) / 2.0;
  double v_angular = (v_right - v_left) / wheel_separation_;

  double delta_s = v_linear * dt;
  double delta_theta = v_angular * dt;

  x_ += delta_s * cos(theta_ + delta_theta / 2.0);
  y_ += delta_s * sin(theta_ + delta_theta / 2.0);
  theta_ += delta_theta;

  // 2. 发布里程计消息
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom.pose.pose.orientation = tf2::toMsg(q);
  
  // 填充协方差矩阵对角线（6x6 矩阵，共 36 个元素）
  // robot_pose_ekf 至少需要 X, Y, Z 和姿态的协方差非零
  for(int i = 0; i < 36; i++) {
    odom.pose.covariance[i] = 0.0;
  }
  odom.pose.covariance[0]  = 0.01;  // X
  odom.pose.covariance[7]  = 0.01;  // Y
  odom.pose.covariance[14] = 0.01;  // Z
  odom.pose.covariance[21] = 0.01;  // Roll
  odom.pose.covariance[28] = 0.01;  // Pitch
  odom.pose.covariance[35] = 0.05;  // Yaw

  // 如果你也发布位姿速度，twist 协方差也建议填充
  for(int i = 0; i < 36; i++) {
    odom.twist.covariance[i] = 0.0;
  }
  odom.twist.covariance[0] = 0.01;
  odom.twist.covariance[35] = 0.05;

  odom_pub_.publish(odom);
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

  // 获取 Z 轴实时角速度
  current_angular_vel_ = msg->angular_velocity.z;
}

void BalanceController::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
  target_linear_vel_ = msg->linear.x;
  target_angular_vel_ = msg->angular.z;
}

void BalanceController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // 这里可以根据需要处理里程计数据
}
} // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerBase)