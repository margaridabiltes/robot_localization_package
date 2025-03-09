#include "robot_localization_package/MyRobotDriver.hpp"

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025


namespace my_robot_driver {

WbNodeRef robot_node = wb_supervisor_node_get_from_def("MY_ROBOT");

void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");


  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
  );

  tf_broadcaster_relative = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  tf_broadcaster_real = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

}

void MyRobotDriver::step() {
  
  const double *position = wb_supervisor_node_get_position(robot_node);
  const double *orientation = wb_supervisor_node_get_orientation(robot_node);

  tf2::Matrix3x3 mat(
      orientation[0], orientation[1], orientation[2],
      orientation[3], orientation[4], orientation[5],
      orientation[6], orientation[7], orientation[8]);

  tf2::Quaternion q;
  mat.getRotation(q);
  double roll, pitch, theta;
  tf2::Matrix3x3(q).getRPY(roll, pitch, theta);

  if(first_update)
  {
    spawn_x = position[0];
    spawn_y = position[1];
    spawn_z = 0;

    spawn_theta = theta;
    spawn_q = q;

    first_update = false;
  }


  double dx = position[0] - spawn_x;  
  double dy = position[1] - spawn_y;

  double relative_x = std::cos(spawn_theta) * dx + std::sin(spawn_theta) * dy;
  double relative_y = -std::sin(spawn_theta) * dx + std::cos(spawn_theta) * dy;
  double relative_z = 0;

  tf2::Quaternion relative_q = spawn_q.inverse() * q;

  //printf("Relative position: %f %f %f\n", relative_x, relative_y, relative_z);
  //printf("Relative orientation: %f %f %f\n", roll, pitch, theta);

  geometry_msgs::msg::TransformStamped tf_real;
  tf_real.header.stamp = rclcpp::Clock().now();
  tf_real.header.frame_id = "map";
  tf_real.child_frame_id = "base_link_real";
  tf_real.transform.translation.x = position[0];
  tf_real.transform.translation.y = position[1];
  tf_real.transform.translation.z = position[2];
  tf_real.transform.rotation.x = q.x();
  tf_real.transform.rotation.y = q.y();
  tf_real.transform.rotation.z = q.z();
  tf_real.transform.rotation.w = q.w();
  tf_broadcaster_real->sendTransform(tf_real);
  //printf("position: %f %f %f\n", position[0], position[1], position[2]);


  geometry_msgs::msg::TransformStamped tf_relative;
  tf_relative.header.stamp = rclcpp::Clock().now();
  tf_relative.header.frame_id = "odom";
  tf_relative.child_frame_id = "base_link";
  tf_relative.transform.translation.x = relative_x;
  tf_relative.transform.translation.y = relative_y;
  tf_relative.transform.translation.z = relative_z;
  tf_relative.transform.rotation.x = relative_q.x();
  tf_relative.transform.rotation.y = relative_q.y();
  tf_relative.transform.rotation.z = relative_q.z();
  tf_relative.transform.rotation.w = relative_q.w();
  tf_broadcaster_relative->sendTransform(tf_relative);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Clock().now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = relative_x;
  odom.pose.pose.position.y = relative_y;
  odom.pose.pose.position.z = relative_z;
  odom.pose.pose.orientation.x = relative_q.x();
  odom.pose.pose.orientation.y = relative_q.y();
  odom.pose.pose.orientation.z = relative_q.z();
  odom.pose.pose.orientation.w = relative_q.w();
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom_pub_->publish(odom);


  // # Speed command
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);
}

} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)