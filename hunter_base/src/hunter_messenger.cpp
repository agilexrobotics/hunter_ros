/*
 * hunter_messenger.cpp
 *
 * Created on: Jun 01, 2020 15:25
 * Description:
 *
 * Test commands:
 * rostopic pub /cmd_vel geometry_msgs/Twist -r 3 -- '[0.5,0.0,0.0]' '[0.0, 0.0,
 * 0.3872]'
 *
 * rostopic pub /reset_odom_integrator std_msgs/Bool true
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "hunter_base/hunter_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include <cmath>

#include "hunter_base/hunter_messenger.hpp"
#include "hunter_msgs/HunterStatus.h"
#include "hunter_msgs/HunterMotorState.h"
#include "hunter_msgs/HunterBmsStatus.h"

namespace westonrobot {
HunterROSMessenger::HunterROSMessenger(ros::NodeHandle *nh)
    : hunter_(nullptr), nh_(nh) {}

HunterROSMessenger::HunterROSMessenger(HunterBase *hunter, ros::NodeHandle *nh)
    : hunter_(hunter), nh_(nh) {}

void HunterROSMessenger::SetupSubscription() {
  // odometry publisher
  odom_publisher_ = nh_->advertise<nav_msgs::Odometry>(odom_frame_, 50);
  status_publisher_ =
      nh_->advertise<hunter_msgs::HunterStatus>("/hunter_status", 10);
  BMS_status_publisher_ = nh_->advertise<hunter_msgs::HunterBmsStatus>("/BMS_status", 10);

  // cmd subscriber
  motion_cmd_subscriber_ = nh_->subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 5, &HunterROSMessenger::TwistCmdCallback, this);
  integrator_reset_subscriber_ = nh_->subscribe<std_msgs::Bool>(
      "/reset_odom_integrator", 5,
      &HunterROSMessenger::ResetOdomIntegratorCallback, this);
}

void HunterROSMessenger::ResetOdomIntegratorCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) ResetOdometry();
}

void HunterROSMessenger::TwistCmdCallback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  double steer_cmd = msg->angular.z;
  if(steer_cmd > HunterParams::max_steer_angle_central)
    steer_cmd = HunterParams::max_steer_angle_central;
  if(steer_cmd < - HunterParams::max_steer_angle_central)
      steer_cmd = - HunterParams::max_steer_angle_central;

  // TODO add cmd limits here
  if (!simulated_robot_) {
    double phi_i = ConvertCentralAngleToInner(msg->angular.z);
    // std::cout << "set steering angle: " << phi_i << std::endl;
    hunter_->SetMotionCommand(msg->linear.x, steer_cmd, phi_i);
  } else {
    std::lock_guard<std::mutex> guard(twist_mutex_);
    current_twist_ = *msg.get();
  }
  // ROS_INFO("cmd received:%f, %f", msg->linear.x, msg->angular.z);
}

void HunterROSMessenger::GetCurrentMotionCmdForSim(double &linear,
                                                   double &angular) {
  std::lock_guard<std::mutex> guard(twist_mutex_);
  linear = current_twist_.linear.x;
  angular = current_twist_.angular.z;
}

double HunterROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = angle;
  if (phi_i > steer_angle_tolerance) {
    // left turn
    double r = l / std::tan(phi_i) + w / 2;
    phi = std::atan(l / r);
  } else if (phi_i < -steer_angle_tolerance) {
    // right turn
    double r = l / std::tan(-phi_i) + w / 2;
    phi = std::atan(l / r);
    phi = -phi;
  }
  return phi;
}

double HunterROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = angle;
  double phi_i = 0;
  if (phi > steer_angle_tolerance) {
    // left turn
    phi_i = std::atan(2 * l * std::sin(phi) /
                      (2 * l * std::cos(phi) - w * std::sin(phi)));
  } else if (phi < -steer_angle_tolerance) {
    // right turn
    phi = -phi;
    phi_i = std::atan(2 * l * std::sin(phi) /
                      (2 * l * std::cos(phi) - w * std::sin(phi)));
    phi_i = -phi_i;
  }
  return phi_i;
}

void HunterROSMessenger::PublishStateToROS() {
  current_time_ = ros::Time::now();
  double dt = (current_time_ - last_time_).toSec();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = hunter_->GetHunterState();

  // publish hunter state message
  hunter_msgs::HunterStatus status_msg;
  hunter_msgs::HunterBmsStatus bms_msg;

  status_msg.header.stamp = current_time_;

  double left_vel = -state.actuator_states[2].motor_rpm / 60.0 * 2 * M_PI /
                    HunterParams::transmission_reduction_rate *
                    HunterParams::wheel_radius;
  double right_vel = state.actuator_states[1].motor_rpm / 60.0 * 2 * M_PI /
                     HunterParams::transmission_reduction_rate *
                     HunterParams::wheel_radius;
  status_msg.linear_velocity = (left_vel + right_vel) / 2.0;

  // TODO SHOULD NOT use this correction when new Firmware with right 
  //     cmd/feedback of steering angle is updated
  // double corrected_angle = state.steering_angle * 26.5 / 40.0;
  // double phi = ConvertInnerAngleToCentral(corrected_angle);
  double phi = ConvertInnerAngleToCentral(state.steering_angle);
  status_msg.steering_angle = phi;
  status_msg.base_state = state.base_state;
  status_msg.control_mode = state.control_mode;
  status_msg.park_mode = state.park_mode;
  status_msg.fault_code = state.fault_code;
  status_msg.battery_voltage = state.battery_voltage;
  for (int i = 0; i < 3; ++i) {
    status_msg.motor_states[i].current = state.actuator_states[i].motor_current;
    status_msg.motor_states[i].rpm = state.actuator_states[i].motor_rpm;
    status_msg.motor_states[i].motor_pose = state.actuator_states[i].motor_pulses;
    status_msg.motor_states[i].temperature = state.actuator_states[i].motor_temperature;
    status_msg.driver_states[i].driver_state = state.actuator_states[i].driver_state;
    status_msg.driver_states[i].driver_voltage = state.actuator_states[i].driver_voltage;
    status_msg.driver_states[i].driver_temperature = state.actuator_states[i].driver_temperature;
  }
  bms_msg.SOC                   = state.SOC;
  bms_msg.SOH                   = state.SOH;
  bms_msg.Alarm_Status_1        = state.Alarm_Status_1;
  bms_msg.Alarm_Status_2        = state.Alarm_Status_2;
  bms_msg.Warning_Status_1      = state.Warning_Status_1;
  bms_msg.Warning_Status_2      = state.Warning_Status_2;
  bms_msg.battery_voltage       = state.bms_battery_voltage;
  bms_msg.battery_current       = state.battery_current;
  bms_msg.battery_temperature   = state.battery_temperature;

  BMS_status_publisher_.publish(bms_msg);
  status_publisher_.publish(status_msg);

  // publish odometry and tf
  PublishOdometryToROS(state.linear_velocity, status_msg.steering_angle, dt);

  // record time for next integration
  last_time_ = current_time_;
}

void HunterROSMessenger::PublishSimStateToROS(double linear, double angular) {
  current_time_ = ros::Time::now();
  double dt = 1.0 / sim_control_rate_;

  // publish hunter state message
  hunter_msgs::HunterStatus status_msg;

  status_msg.header.stamp = current_time_;

  // TODO should receive update from simulator
  status_msg.linear_velocity = linear;
  status_msg.steering_angle = angular;

  status_msg.base_state = 0x00;
  status_msg.control_mode = 0x01;
  status_msg.fault_code = 0x00;
  status_msg.battery_voltage = 29.5;

  for (int i = 0; i < 3; ++i) {
    status_msg.motor_states[i].current = 0;
    status_msg.motor_states[i].rpm = 0;
    status_msg.motor_states[i].temperature = 0;
  }

  status_publisher_.publish(status_msg);

  // publish odometry and tf
  PublishOdometryToROS(linear, angular, dt);
}

void HunterROSMessenger::ResetOdometry() {
  position_x_ = 0.0;
  position_y_ = 0.0;
  theta_ = 0.0;
}

void HunterROSMessenger::PublishOdometryToROS(double linear, double angular,
                                              double dt) {
  // perform numerical integration to get an estimation of pose
  linear_speed_ = linear;
  steering_angle_ = angular;

  // propagate model model
  asc::state_t state =
      model_.Propagate({position_x_, position_y_, theta_},
                       {linear_speed_, steering_angle_}, 0, dt, dt / 100);
  position_x_ = state[0];
  position_y_ = state[1];
  theta_ = state[2];

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  // publish tf transformation
  if (publish_tf_) {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(tf_msg);
  }

  // publish odometry and tf messages
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = linear_speed_;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = linear_speed_ / l * std::tan(steering_angle_);

  odom_msg.pose.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
  odom_msg.twist.covariance = {
    0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
    0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
    0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
    0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
    0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
    0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};

//   std::cerr << "linear: " << linear_speed_ << " , angular: " << steering_angle_
//             << " , pose: (" << position_x_ << "," << position_y_ << ","
//             << theta_ << ")" << std::endl;

  odom_publisher_.publish(odom_msg);
}
}  // namespace westonrobot
