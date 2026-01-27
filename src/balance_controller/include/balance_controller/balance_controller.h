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
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include "balance_controller/filters/filters.h"

namespace balance_controller {

class VelocityKalman {
public:
    VelocityKalman(double Q, double R, double P) : Q_(Q), R_(R), P_(P), v_est_(0.0) {}

    double update(double v_meas, double dt, double effort) {
        // 1. 预测 (Predict)
        // 假设加速度项由 effort 决定，a_coeff 需要根据物理模型微调，也可设为 0 纯靠模型自校正
        double a_coeff = 0.5; 
        double v_pred = v_est_ + (a_coeff * effort) * dt;
        P_ = P_ + Q_;

        // 2. 更新 (Update)
        double K = P_ / (P_ + R_);
        v_est_ = v_pred + K * (v_meas - v_pred);
        P_ = (1 - K) * P_;

        return v_est_;
    }

private:
    double Q_; // 过程噪声：相信模型的程度
    double R_; // 测量噪声：相信编码器的程度
    double P_; // 估计误差协方差
    double v_est_; // 估计的速度值
};

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
  ros::Publisher pos_error_pub_;
  ros::Publisher vel_error_pub_;
  ros::Publisher pitch_error_pub_;
  ros::Publisher omega_error_pub_;
  ros::Publisher last_effort_pub_;

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

  control_toolbox::Pid yaw_pid_;

  double current_pitch_ = 0.0; // 当前角度
  double current_pos_ = 0.0; // 当前位移
  double current_linear_vel_ = 0.0; // 当前速度
  double current_angular_vel_ = 0.0; // 当前角速度
  double current_omega_ = 0.0; // 当前角速度

  double target_pitch_ = 0.0;
  double target_pos_ = 0.0;
  double target_linear_vel_ = 0.0;
  double target_angular_vel_ = 0.0;
  double target_omega_ = 0.0;

  double wheel_separation_ = 0.53; // 轮距
  double wheel_radius_ = 0.1;      // 轮子半径

  double last_effort_ = 0.0; // 上一次控制输出
  double k1, k2, k3, k4; // LQR 控制器增益
  VelocityKalman vel_kf{0.01, 0.1, 1.0};
};


} // namespace balance_controller