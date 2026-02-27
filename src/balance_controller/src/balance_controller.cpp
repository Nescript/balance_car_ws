#include "balance_controller/balance_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <boost/concept_check.hpp>
#include "balance_controller/filters/math_utilities.h"

namespace balance_controller {

bool BalanceController::init(hardware_interface::PositionJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  try {
    left_l1_joint_ = effort_joint_interface->getHandle("left_hip_joint");
    right_l1_joint_ = effort_joint_interface->getHandle("right_hip_joint");
    left_l4_joint_ = effort_joint_interface->getHandle("left_linkage_2_joint");
    right_l4_joint_ = effort_joint_interface->getHandle("right_linkage_2_joint");
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR("Could not get joint handles: %s", e.what());
    return false;
  }
  if (!controller_nh.getParam("l1", l1_) ||
      !controller_nh.getParam("l2", l2_) ||
      !controller_nh.getParam("l3", l3_) ||
      !controller_nh.getParam("l4", l4_) ||
      !controller_nh.getParam("l5", l5_)) {
    ROS_ERROR("Failed to load link lengths from parameter server");
    return false;
  }
  gimbal_imu_sub_ = root_nh.subscribe("gimbal_imu", 1, &BalanceController::gimbalImuCallback, this);
  controller_nh.param("wheel_separation", wheel_separation_, 0.2);
  controller_nh.param("wheel_radius", wheel_radius_, 0.03);
  pos_error_pub_ = root_nh.advertise<std_msgs::Float64>("position_error", 1);
  vel_error_pub_ = root_nh.advertise<std_msgs::Float64>("velocity_error", 1);
  pitch_error_pub_ = root_nh.advertise<std_msgs::Float64>("pitch_error", 1);
  omega_error_pub_ = root_nh.advertise<std_msgs::Float64>("omega_error", 1);
  last_effort_pub_ = root_nh.advertise<std_msgs::Float64>("last_effort", 1);
  current_left_point_pos_ = root_nh.advertise<geometry_msgs::Point>("current_left_point", 1);
  current_right_point_pos_ = root_nh.advertise<geometry_msgs::Point>("current_right_point", 1);
  target_left_point_pos_ = root_nh.advertise<geometry_msgs::Point>("target_left_point", 1);
  target_right_point_pos_ = root_nh.advertise<geometry_msgs::Point>("target_right_point", 1);

  controller_nh.param("traj_x_center",      traj_x_center_,     l5_ / 2.0);   // 基座中心
  controller_nh.param("traj_y_center",      traj_y_center_,     0.22);        // 工作区中部
  controller_nh.param("traj_x_amplitude",   traj_x_amplitude_,  0.03);
  controller_nh.param("traj_y_amplitude",   traj_y_amplitude_,  0.02);
  controller_nh.param("traj_frequency",     traj_frequency_,    0.5);         // 0.5 Hz
  controller_nh.param("traj_sine_periods",  traj_sine_periods_, 2);           // 2个正弦半周期

  imu_sub_ = root_nh.subscribe("imu", 1, &BalanceController::imuCallback, this);
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &BalanceController::cmdVelCallback, this);

  return true;
}

void BalanceController::starting(const ros::Time& time) {
  // 重置轨迹时钟，确保从当前时刻开始
  start_time_ = time;

  // 读取当前关节角作为初始保持位置，避免启动瞬间跳变
  double init_theta1_l = left_l1_joint_.getPosition();
  double init_theta4_l = left_l4_joint_.getPosition();
  double init_theta1_r = right_l1_joint_.getPosition();
  double init_theta4_r = right_l4_joint_.getPosition();
  left_l1_joint_.setCommand(init_theta1_l);
  left_l4_joint_.setCommand(init_theta4_l);
  right_l1_joint_.setCommand(init_theta1_r);
  right_l4_joint_.setCommand(init_theta4_r);

  ROS_INFO("BalanceController started. Init angles: l1_L=%.3f l4_L=%.3f l1_R=%.3f l4_R=%.3f",
           init_theta1_l, init_theta4_l, init_theta1_r, init_theta4_r);

  target_linear_vel_ = 0.0;
  target_angular_vel_ = 0.0;
  current_pos_ = 0.0;
  target_pos_ = 0.0;
  current_state_ = STATE_NORMAL;
}

void BalanceController::update(const ros::Time& time, const ros::Duration& period) {
  // 1. 生成正弦轨迹目标末端点 (x, y)
  double target_x, target_y;
  sinusoidalTrajectory(time, target_x, target_y);
  
  // 2. 逆运动学 → 目标关节角
  double target_theta1, target_theta4;
  if (!inverseKinematics(target_x, target_y, target_theta1, target_theta4)) {
    ROS_WARN_THROTTLE(1.0, "IK failed for target (%.4f, %.4f), holding position",
                     target_x, target_y);
    return;
  }

  // 3. 读取当前各关节实际角度
  double cur_l1_left  = left_l1_joint_.getPosition();
  double cur_l4_left  = left_l4_joint_.getPosition();
  double cur_l1_right = right_l1_joint_.getPosition();
  double cur_l4_right = right_l4_joint_.getPosition();

  // 4. 直接下发目标位置（PositionJointInterface 由底层 PID 伺服）
  left_l1_joint_.setCommand(target_theta1);
  left_l4_joint_.setCommand(target_theta4);
  right_l1_joint_.setCommand(target_theta1);
  right_l4_joint_.setCommand(target_theta4);

  // 5. 正解验证：计算实际末端位置（用于调试）
  double fk_x, fk_y;
  if (forwardKinematics(cur_l1_left, cur_l4_left, fk_x, fk_y)) {
    ROS_INFO_THROTTLE(0.5,
      "Traj target=(%.4f,%.4f) | FK actual=(%.4f,%.4f) | theta1=%.4f->%.4f theta4=%.4f->%.4f",
      target_x, target_y, fk_x, fk_y,
      cur_l1_left, target_theta1, cur_l4_left, target_theta4);
  }

  geometry_msgs::Point tgt_point, current_point;
  tgt_point.x = target_x;
  tgt_point.y = target_y;
  tgt_point.z = 0.0;
  target_left_point_pos_.publish(tgt_point);

  current_point.x = fk_x;
  current_point.y = fk_y;
  current_point.z = 0.0;
  current_left_point_pos_.publish(current_point);
}

bool BalanceController::inverseKinematics(double cx, double cy,
                                              double &theta1, double &theta4) {
  double dist_sq_ac = cx * cx + cy * cy;
  double dist_ac    = sqrt(dist_sq_ac);
  if (dist_ac < 1e-10) {
    ROS_WARN("IK: target too close to joint A");
    return false;
  }
  double cos_alpha = (l1_ * l1_ + dist_sq_ac - l2_ * l2_) / (2.0 * l1_ * dist_ac);
  if (cos_alpha < -1.001 || cos_alpha > 1.001) {
    ROS_WARN("IK: left chain unreachable (cos_alpha=%.4f)", cos_alpha);
    return false;
  }
  cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));
  double gamma_l  = atan2(cy, cx);
  double alpha_l  = acos(cos_alpha);
  double phi1     = gamma_l + alpha_l;
  double dx_ec    = cx - l5_;
  double dy_ec    = cy; 
  double dist_sq_ec = dx_ec * dx_ec + dy_ec * dy_ec;
  double dist_ec    = sqrt(dist_sq_ec);
  if (dist_ec < 1e-10) {
    ROS_WARN("IK: target too close to joint E");
    return false;
  }
  double cos_beta = (l4_ * l4_ + dist_sq_ec - l3_ * l3_) / (2.0 * l4_ * dist_ec);
  if (cos_beta < -1.001 || cos_beta > 1.001) {
    ROS_WARN("IK: right chain unreachable (cos_beta=%.4f)", cos_beta);
    return false;
  }
  cos_beta = std::max(-1.0, std::min(1.0, cos_beta));
  double gamma_r  = atan2(dy_ec, dx_ec);
  double alpha_r  = acos(cos_beta);
  double phi4     = gamma_r - alpha_r;

  theta1 = phi1 - M_PI;
  theta4 = phi4 - M_PI / 2;

  return true;
}

bool BalanceController::forwardKinematics(double theta1, double theta4,
                                              double &px, double &py) {

  double phi1 = theta1 + M_PI;
  double phi4 = theta4 + M_PI / 2;
  double bx = l1_ * cos(phi1);
  double by = l1_ * sin(phi1);
  double dx_elbow = l5_ + l4_ * cos(phi4); 
  double dy_elbow = l4_ * sin(phi4);
  double dist_x = dx_elbow - bx;
  double dist_y = dy_elbow - by;
  double d      = sqrt(dist_x * dist_x + dist_y * dist_y);

  if (d > l2_ + l3_ + 1e-6 || d < fabs(l2_ - l3_) - 1e-6 || d < 1e-10) {
    ROS_WARN("FK: no solution (d=%.4f, valid=[%.4f, %.4f])",
             d, fabs(l2_ - l3_), l2_ + l3_);
    return false;
  }
  double a    = (l2_ * l2_ - l3_ * l3_ + d * d) / (2.0 * d);
  double h_sq = l2_ * l2_ - a * a;
  if (h_sq < 0.0) h_sq = 0.0;
  double h = sqrt(h_sq);
  double mx = bx + a * dist_x / d;
  double my = by + a * dist_y / d;
  double c1x = mx + h * (-dist_y) / d;
  double c1y = my + h * (dist_x)  / d;
  double c2x = mx - h * (-dist_y) / d;
  double c2y = my - h * (dist_x)  / d;

  if (c1y >= c2y) { px = c1x; py = c1y; }
  else            { px = c2x; py = c2y; }

  return true;
}

void BalanceController::sinusoidalTrajectory(const ros::Time &time,
                                                  double &target_x, double &target_y) {
  double elapsed = (time - start_time_).toSec();
  double phase = elapsed * traj_frequency_;
  double s = 2.0 * fabs(fmod(fabs(phase), 2.0) - 1.0) - 1.0;
  target_x = traj_x_center_ + traj_x_amplitude_ * s;
  target_y = traj_y_center_ + traj_y_amplitude_ * sin(traj_sine_periods_ * M_PI * s);
}

void BalanceController::stopping(const ros::Time& time) {
  // 位置接口：停止时保持当前关节角不动
  left_l1_joint_.setCommand(left_l1_joint_.getPosition());
  left_l4_joint_.setCommand(left_l4_joint_.getPosition());
  right_l1_joint_.setCommand(right_l1_joint_.getPosition());
  right_l4_joint_.setCommand(right_l4_joint_.getPosition());
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