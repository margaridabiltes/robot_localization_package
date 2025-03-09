#include "robot_localization_package/fake_feature_extractor.hpp"

KeypointDetector::KeypointDetector() 
    : Node("keypoint_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    
    keypoints_ = {{-0.75, 0.75}, {0.75, 0.75}, {0.75, -0.75}, {-0.75, -0.75}};

    keypoint_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/keypoints", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&KeypointDetector::checkAndPublishKeypoints, this));

    last_x_ = 0.0;
    last_y_ = 0.0;
    last_theta_ = 0.0;
}

void KeypointDetector::checkAndPublishKeypoints() {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("base_link_real", "map", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(get_logger(), "Could not get robot transform: %s", ex.what());
        return;
    }

    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;

    printf("Robot position: %f %f\n", x, y);

    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );
    double roll, pitch, theta;
    tf2::Matrix3x3(q).getRPY(roll, pitch, theta);

    // Check if movement is significant before publishing keypoints
    if (std::hypot(x - last_x_, y - last_y_) > 0.01 || std::abs(theta - last_theta_) > 0.01) {
        printf("Publishing keypoints\n");
        publishTransformedKeypoints(transform);
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;
    }
}

void KeypointDetector::publishTransformedKeypoints(const geometry_msgs::msg::TransformStamped& transform) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = "base_link_real";
    cloud_msg.height = 1;
    cloud_msg.width = keypoints_.size();
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(3, 
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32
    );

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto& kp : keypoints_) {
        geometry_msgs::msg::Point point;
        point.x = kp.first;
        point.y = kp.second;
        point.z = 0.0;

        geometry_msgs::msg::Point transformed_point;
        tf2::doTransform(point, transformed_point, transform);

        *iter_x = transformed_point.x;
        *iter_y = transformed_point.y;
        *iter_z = transformed_point.z;

        ++iter_x; ++iter_y; ++iter_z;
    }

    keypoint_pub_->publish(cloud_msg);
    RCLCPP_INFO(get_logger(), "Published transformed keypoints.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeypointDetector>());
    rclcpp::shutdown();
    return 0;
}
