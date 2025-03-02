#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class KeypointDetector : public rclcpp::Node {
public:
    KeypointDetector() : Node("keypoint_detector") {
        // Predefined keypoints in the "map" frame
        keypoints_ = {
            {-0.75, 0.75}, {0.75, 0.75}, {0.75, -0.75}, {-0.75, -0.75}
        };

        // Initialize TF2 listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher for transformed keypoints as a PointCloud2
        keypoint_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/transformed_keypoints", 10
        );

        // Timer to update keypoints at 1Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&KeypointDetector::publishTransformedKeypoints, this)
        );
    }

private:
    std::vector<std::pair<double, double>> keypoints_;  // Keypoints (x, y)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keypoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishTransformedKeypoints() {
        geometry_msgs::msg::TransformStamped transform;

        try {
            // Get the transform from "map" to "base_link"
            transform = tf_buffer_->lookupTransform(
                "base_link", "odom", tf2::TimePointZero
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform keypoints: %s", ex.what());
            return;
        }

        // Create a PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = "base_link";

        // Set PointCloud2 fields
        cloud_msg.height = 1; // Unordered point cloud
        cloud_msg.width = keypoints_.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        // Define fields (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(
            3, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32
        );

        // Fill point cloud data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto &kp : keypoints_) {
            auto transformed_kp = transformPoint(kp, transform);

            *iter_x = transformed_kp.first;
            *iter_y = transformed_kp.second;
            *iter_z = 0.0; // No Z component

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        // Publish the transformed keypoints as a PointCloud2
        keypoint_pub_->publish(cloud_msg);
    }

    std::pair<double, double> transformPoint(
        const std::pair<double, double> &point,
        const geometry_msgs::msg::TransformStamped &transform
    ) {
        double x_map = point.first;
        double y_map = point.second;

        // Extract translation
        double tx = transform.transform.translation.x;
        double ty = transform.transform.translation.y;

        // Extract rotation (quaternion)
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );

        // Convert quaternion to 2D yaw angle
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Apply 2D transformation (rotation + translation)
        double x_base = std::cos(yaw) * (x_map - tx) + std::sin(yaw) * (y_map - ty);
        double y_base = -std::sin(yaw) * (x_map - tx) + std::cos(yaw) * (y_map - ty);

        return {x_base, y_base};
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeypointDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
