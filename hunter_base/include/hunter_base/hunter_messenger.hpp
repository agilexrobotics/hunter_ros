/*
 * hunter_messenger.hpp
 *
 * Created on: Jun 01, 2020 15:18
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <string>

#include <tf2_ros/transform_broadcaster.h>

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include <mutex>
#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/bicycle_model.hpp"
#include "hunter_base/hunter_params.hpp"


namespace westonrobot {
template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
      // Note: you may need to add additional constraints to [x]
    }

    return x;
  }

 private:
  asc::RK4 integrator_;
};

class HunterROSMessenger {
 public:
  explicit HunterROSMessenger(ros::NodeHandle *nh);
  HunterROSMessenger(HunterRobot *hunter, ros::NodeHandle *nh);

  std::string odom_frame_;
  std::string base_frame_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;
  bool publish_tf_ = true;
  int version = 2;

  void SetupSubscription();
  void ResetOdometry();

  void PublishStateToROS();
  void PublishSimStateToROS(double linear, double angular);

  void GetCurrentMotionCmdForSim(double &linear, double &angular);
  void SetWeelbase(float Weelbase){
    l = Weelbase;
  }
  void SetTrack(float Track){
    w = Track;
  }
  void SetMaxSteerAngleCentral(float Angle){
    max_steer_angle_central = Angle;
  }
 private:
  HunterRobot *hunter_;
  ros::NodeHandle *nh_;

  std::mutex twist_mutex_;
  geometry_msgs::Twist current_twist_;

  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher BMS_status_publisher_;
  ros::Subscriber motion_cmd_subscriber_;
  ros::Subscriber integrator_reset_subscriber_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // control inputs
  double linear_speed_ = 0.0;
  double steering_angle_ = 0.0;

//  static constexpr double l = HunterV2Params::wheelbase;
//  static constexpr double w = HunterV2Params::track;
  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees
  double l = 0.0;
  double w = 0.0;
  double max_steer_angle_central = 0.0;
  // state variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  SystemPropagator<BicycleKinematics> model_;

  ros::Time last_time_;
  ros::Time current_time_;

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

  void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void ResetOdomIntegratorCallback(const std_msgs::Bool::ConstPtr &msg);
  void PublishOdometryToROS(double linear, double angular, double dt);
};
}  // namespace westonrobot

#endif /* HUNTER_MESSENGER_HPP */
