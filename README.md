# robot_localization_package
## Description
* ROS2 Particle Filter implementation to be used together with packages:
    * [robot_worlds](https://github.com/JMCOliveira02/robot_worlds)
    * [robot_msgs](https://github.com/JMCOliveira02/robot_msgs)
## Installation
* Clone this repository into your ROS2 workspace src folder
# Usage
## Subscribed topics (particle_filter node)
* **/odom (nav_msgs::msg::Odometry)** -> The estimated odometry of the robot. This topic is used in the prediction step of the filter. 
* **/corner (robot_msgs::msg::FeatureArray)** -> The detected features' positions and orientations, from the robot's reference frame. This topic is used in the measurement step of the filter.
## Published topics 
## Launch files
