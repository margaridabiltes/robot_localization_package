#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include <cstdio>
#include <functional>
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/msg/odometry.h>

#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/transform_broadcaster.h"




namespace my_robot_driver {
class MyRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
      
  geometry_msgs::msg::Twist cmd_vel_msg;

  
  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
  WbDeviceTag right_encoder;
  WbDeviceTag left_encoder;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_real;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_relative;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;

  double spawn_x = 0.0;
  double spawn_y = 0.0;
  double spawn_z = 0.0;
  double spawn_theta = 0.0;

  tf2::Quaternion spawn_q;

  bool first_update = true;

  double est_x = 0.0;
  double est_y = 0.0;
  double est_theta = 0.0;
  double last_left_wheel_pos = 0.0;
  double last_right_wheel_pos = 0.0;


};
} // namespace my_robot_driver
#endif