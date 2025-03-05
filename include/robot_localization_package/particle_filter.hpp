#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.hpp> 
#include <geometry_msgs/msg/pose_array.hpp>
#include <vector>
#include <random>

class ParticleFilter : public rclcpp::Node {
public:
    ParticleFilter();

private:
    struct Particle {
        double x, y, theta;  // Particle state (position and orientation)
        double weight;       // Importance weight
    };

    int num_particles_; // Number of particles
    std::vector<Particle> particles_; // Particle set

    // ROS2 Publishers & Subscribers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keypoint_sub_;

    // ✅ TF2 for transformation handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Motion Tracking (stores last known odometry values)
    double last_odom_x_, last_odom_y_, last_odom_theta_;

    // Random generator for resampling
    std::default_random_engine generator_;

    // Initialize particles randomly
    void initializeParticles();

    // Apply motion model to particles
    
    void motionUpdate();
    rclcpp::TimerBase::SharedPtr motion_timer_;  


    // Update particle weights based on feature matching
    void measurementUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    double sensor_noise_ = 0.1; 

    // Resample particles based on their weights
    void resampleParticles();

    // Compute and publish the estimated pose
    void publishEstimatedPose();

    void publishParticles();

    // Get expected features for a given particle
    std::vector<std::pair<double, double>> getExpectedFeatures(const Particle &p);
};

#endif // PARTICLE_FILTER_HPP
