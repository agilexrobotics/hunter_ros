#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "hunter_base/hunter_messenger.hpp"

using namespace westonrobot;

HunterBase robot;

void DetachRobot(int signal) {
  robot.Disconnect();
  robot.Terminate();
}

int main(int argc, char **argv) {
  // setup ROS node
  ros::init(argc, argv, "hunter_odom");
  ros::NodeHandle node(""), private_node("~");

  std::signal(SIGINT, DetachRobot);

  // instantiate a robot object
  HunterBase robot;
  HunterROSMessenger messenger(&robot, &node);

  // fetch parameters before connecting to robot
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, true);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);

  // no connection for simulated robot
  // setup ROS subscription
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  double linear, angular;
  ros::Rate rate_50hz(50);  // 50Hz
  while (ros::ok()) {
    messenger.GetCurrentMotionCmdForSim(linear, angular);
    messenger.PublishSimStateToROS(linear, angular);
    ros::spinOnce();
    rate_50hz.sleep();
  }

  return 0;
}
