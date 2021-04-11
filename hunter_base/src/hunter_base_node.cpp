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
  ros::init(argc, argv, "hunter_base");
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
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_,
                           false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<bool>("publish_tf", messenger.publish_tf_,  true);

  // connect to robot and setup ROS subscription
  if (port_name.find("can") != std::string::npos) {
    robot.Connect(port_name);
    ROS_INFO("Using CAN bus to talk with the robot: %s", port_name.c_str());
  } else {
    ROS_ERROR("Only CAN bus interface is supported for now");
    return -1;
  }
  messenger.SetupSubscription();

  // publish robot state at 50Hz while listening to twist commands
  ros::Rate rate_50hz(50);  // 50Hz
  //int cnt = 0;
  while (ros::ok()) {
    messenger.PublishStateToROS();
    ros::spinOnce();
    rate_50hz.sleep();
    // if(++cnt == 390) break;
  }

  return 0;
}
