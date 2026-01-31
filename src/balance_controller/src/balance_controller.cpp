#include "balance_controller/balance_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <boost/concept_check.hpp>
#include "balance_controller/filters/math_utilities.h"

namespace balance_controller {

bool BalanceController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  try {
    left_wheel_joint_ = effort_joint_interface->getHandle("left_wheel_joint");
    right_wheel_joint_ = effort_joint_interface->getHandle("right_wheel_joint");
    gimbal_pitch_joint_ = effort_joint_interface->getHandle("muzzle_joint");
    gimbal_yaw_joint_ = effort_joint_interface->getHandle("gimbal_joint");
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR("Could not get joint handles: %s", e.what());
    return false;
  }
  if (!yaw_pid_.init(ros::NodeHandle(controller_nh, "yaw_pid"))) {
    ROS_ERROR("Failed to initialize yaw PID");
    return false;
  }
  if (!gimbal_pitch_pid_.init(ros::NodeHandle(controller_nh, "gimbal_pitch_pid"))) {
    ROS_ERROR("Failed to initialize gimbal pitch PID");
    return false;
  }
  if (!gimbal_yaw_pid_.init(ros::NodeHandle(controller_nh, "gimbal_yaw_pid"))) {
    ROS_ERROR("Failed to initialize gimbal yaw PID");
    return false;
  }
  gimbal_imu_sub_ = root_nh.subscribe("gimbal_imu", 1, &BalanceController::gimbalImuCallback, this);
  std::vector<double> q_list, r_list;
  if (controller_nh.getParam("lqr/q", q_list)) {
    if (q_list.size() == 16) {
      Q_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(q_list.data());
    } else {
      ROS_ERROR("Q list size should be 16");
      return false;
    }
  }

  // 读取 R 矩阵
  if (controller_nh.getParam("lqr/r", r_list)) {
    if (r_list.size() == 1) {
      R_(0, 0) = r_list[0];
    } else {
      ROS_ERROR("R list size should be 1");
      return false;
    }
  }
  if (controller_nh.getParam("lqr/q_selfup", q_list)) {
    if (q_list.size() == 16) {
      Q_selfup_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(q_list.data());
    } else {
      ROS_ERROR("Q list size should be 16");
      return false;
    }
  }

  // 读取 R 矩阵
  if (controller_nh.getParam("lqr/r_selfup", r_list)) {
    if (r_list.size() == 1) {
      R_selfup_(0, 0) = r_list[0];
    } else {
      ROS_ERROR("R list size should be 1");
      return false;
    }
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

  A_ << 0.0, 1.0, 0.0, 0.0,
        0.0, -5.42416193e-03, -8.32205709e-01, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, -9.75084900e-03, 1.91311657e+01, 0.0;

  B_ << 0.0,
        2.71208097,
        0.0,
       -4.8754245;
  Lqr<double> lqr(A_, B_, Q_, R_);
  Lqr<double> lqr_selfup(A_, B_, Q_selfup_, R_selfup_);
  if (lqr.computeK()) {
    K_ = lqr.getK();
    ROS_INFO_STREAM("LQR K: " << K_);
  } else {
    ROS_ERROR("Failed to compute LQR K");
    return false;
  }
  if (lqr_selfup.computeK()) {
    K_selfup_ = lqr_selfup.getK();
    ROS_INFO_STREAM("LQR K Self-up: " << K_selfup_);
  } else {
    ROS_ERROR("Failed to compute LQR K selfup");
    return false;
  }
  return true;
}

void BalanceController::starting(const ros::Time& time) {
  target_linear_vel_ = 0.0;
  target_angular_vel_ = 0.0;
  target_pitch_ = -0.1415;
  target_omega_ = 0.0;

  current_pos_ = 0.0;
  target_pos_ = 0.0;
  current_state_ = STATE_NORMAL;
}

void BalanceController::update(const ros::Time& time, const ros::Duration& period) {
  double dt = period.toSec();
  Vec4<double> x_error;
  double current_raw_linear_vel_ = (left_wheel_joint_.getVelocity() + right_wheel_joint_.getVelocity()) * wheel_radius_ / 2.0;
  current_linear_vel_ = vel_kf.update(current_raw_linear_vel_, dt, last_effort_);

  current_pos_ =(left_wheel_joint_.getPosition() + right_wheel_joint_.getPosition()) * wheel_radius_ / 2;
  target_pos_ += target_linear_vel_ * dt;
  
  double pos_error = current_pos_ - target_pos_; 
  double vel_error = current_linear_vel_ - target_linear_vel_;

  double pitch_error = current_pitch_ - target_pitch_;
  double omega_error = current_omega_ - target_omega_;
  x_error << pos_error, vel_error, pitch_error, omega_error; 
    
  double gimbal_yaw_effort = gimbal_yaw_pid_.computeCommand(gimbal_target_angular_vel_ - gimbal_current_angular_vel_, period);
  // to do 云台yaw追速度指令，底盘yaw追imu角度
  gimbal_yaw_joint_.setCommand(gimbal_yaw_effort);
  double yaw_effort = yaw_pid_.computeCommand(angularMinus(gimbal_current_yaw_, chassis_current_yaw_), period);

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
        if ((time - self_up_start_time_).toSec() > 6.0) {
            current_state_ = STATE_NORMAL;
            ROS_INFO("Switched to NORMAL state");
        }
        break;
  }

  double base_effort = 0;
  if (current_state_ == STATE_SELF_UP) {
    // base_effort = -(k1_selfup * pos_error + k2_selfup * vel_error + k3_selfup * pitch_error + k4_selfup * omega_error);
    base_effort = -(K_selfup_ * x_error)(0, 0);
    yaw_effort = 0; // 自起时不进行yaw控制
  }
  else if (current_state_ == STATE_NORMAL) {
    // base_effort = -(k1 * pos_error + k2 * vel_error + k3 * pitch_error + k4 * omega_error);
    base_effort = -(K_ * x_error)(0, 0);
  }
  else {
      base_effort = 0;
      yaw_effort = 0;
  }

  left_wheel_joint_.setCommand(base_effort / 2 - yaw_effort);
  right_wheel_joint_.setCommand(base_effort / 2 + yaw_effort);
  // 倒地时候可以清空这个转向pid的输出
  last_effort_ = base_effort;

  double gimbal_pitch_error = gimbal_target_pitch_ - gimbal_current_pitch_;
  gimbal_pitch_joint_.setCommand(gimbal_pitch_pid_.computeCommand(gimbal_pitch_error, period));

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
  chassis_current_yaw_ = yaw;
  current_omega_ = msg->angular_velocity.y;
  current_angular_vel_ = msg->angular_velocity.z;
}

void BalanceController::cmdVelCallback(const geometry_msgs::TwistConstPtr& msg) {
  target_linear_vel_ = msg->linear.x;
  gimbal_target_angular_vel_ = msg->angular.z;
}

void BalanceController::gimbalImuCallback(const sensor_msgs::ImuConstPtr& msg) {
  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  gimbal_current_pitch_ = pitch;
  gimbal_current_yaw_ = yaw;
  gimbal_current_angular_vel_ = msg->angular_velocity.z;
}
} // namespace balance_controller
PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerBase)