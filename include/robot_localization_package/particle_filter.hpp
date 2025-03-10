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
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <vector>
#include <random>

class ParticleFilter : public rclcpp::Node {
public:
    ParticleFilter();

private:
    struct Particle {
        double x, y, theta; 
        double init_x, init_y, init_theta;
        double weight;      
    };

    bool first_update_ = true;  


    int num_particles_; 
    std::vector<Particle> particles_;
    

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_2;

    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    double last_x_, last_y_, last_theta_;
    double x_last_final = 0.0;
    double y_last_final = 0.0;
    double theta_last_final = 0.0;
    
    std::default_random_engine generator_;

    void initializeParticles();

    void motionUpdate(const nav_msgs::msg::Odometry::SharedPtr msg);

    sensor_msgs::msg::PointCloud2::SharedPtr last_keypoint_msg_; 
    nav_msgs::msg::Odometry::SharedPtr msg_odom_base_link_;

    void storeKeypointMessage(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void storeOdomBaseLink(const nav_msgs::msg::Odometry::SharedPtr msg);
    void measurementUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    double sensor_noise_ = 0.5; 

    void resampleParticles();

    void publishEstimatedPose();

    rclcpp::TimerBase::SharedPtr timer_pose_;

    void computeEstimatedPose();
    
    void publishParticles();

    std::vector<std::pair<double, double>> getExpectedFeatures(const Particle &p);
};
#endif