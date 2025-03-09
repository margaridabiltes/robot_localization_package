#ifndef KEYPOINT_DETECTOR_HPP
#define KEYPOINT_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class KeypointDetector : public rclcpp::Node {
public:
    KeypointDetector();

private:
    std::vector<std::pair<double, double>> keypoints_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keypoint_pub_;

    double last_x_, last_y_, last_theta_;

    void checkAndPublishKeypoints();

    rclcpp::TimerBase::SharedPtr timer_;
    
    void publishTransformedKeypoints(const geometry_msgs::msg::TransformStamped& transform);
};

#endif // KEYPOINT_DETECTOR_HPP
