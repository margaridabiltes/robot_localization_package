#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <robot_msgs/msg/feature.hpp>
#include <robot_msgs/msg/feature_array.hpp>
#include "robot_localization_package/FeatureStruct.hpp"
#include "robot_localization_package/MapLoader.hpp"
#include <vector>
#include <random>
#include <fstream>
#include <array>
#include <string>

#define NUM_PARTICLES 1000.0

#define ROOM_SIZE_X 4.0
#define ROOM_SIZE_Y 4.0
#define ROOM_SIZE_Z 0.0

#define MOTION_DELTA_DISTANCE 0.1
#define MOTION_DELTA_ANGLE 0.1
#define MOTION_X_VARIANCE 0.05
#define MOTION_Y_VARIANCE 0.05
#define MOTION_ANGLE_VARIANCE 0.08

#define RESAMPLE_ESS_THRESHOLD 0.5
#define RESAMPLE_MAX_WEIGHT_THRESHOLD 4.0

#define INJECT_NUM_ITERATIONS 10
#define INJECT_PERCENTAGE 0.3

#define REPLACE_WORST_PERCENTAGE 0.15

#define ESTIMATE_NUM_PARTICLES 10


class ParticleFilter : public rclcpp::Node {
public:
    // Constructor
    ParticleFilter();

private:
    // Particle structure
    struct Particle {
        double x, y, theta;  // Position and orientation
        double weight;       // Weight of the particle
    };

    // Decoded message structure
    struct DecodedMsg {
        double x, y, z, theta;  // Position and orientation
        std::string type;       // Feature type
        double confidence;      // Confidence level of classification
        std::array<std::array<double, 3>, 3> covariance_pos;   // Position covariance
        std::array<std::array<double, 3>, 3> covariance_angle; // Orientation covariance
    };

    // Resampling methods
    enum class ResamplingMethod {
        MULTINOMIAL,
        STRATIFIED,
        SYSTEMATIC,
        RESIDUAL
    };

    // Resampling triggers
    enum class ResamplingAmount {
        ESS,
        MAX_WEIGHT
    };

    // Random number generator
    std::default_random_engine generator_;

    // Map loader and features
    map_features::MapLoader map_loader_;
    std::vector<map_features::FeaturePtr> global_features_;
    std::string map_features_;
    double room_size_x_, room_size_y_;
    
    // Particle filter variables
    double num_particles_;
    std::vector<Particle> particles_;
    bool resample_flag_ = false;
    bool new_map = false;
    double motion_delta_distance_, motion_delta_angle_;
    double motion_x_variance_, motion_y_variance_, motion_angle_variance_;
    double resample_ess_threshold_;
    double resample_max_weight_threshold_;
    int inject_num_iterations_;
    double inject_percentage_;
    double replace_worst_percentage_;
    int estimate_num_particles_;

    // Logging
    std::ofstream log_file_;

    // ROS2 publishers, subscribers, and timers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr particles_pub_;
    rclcpp::Subscription<robot_msgs::msg::FeatureArray>::SharedPtr feature_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_pose_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Last received messages
    robot_msgs::msg::FeatureArray::SharedPtr last_map_msg_;
    nav_msgs::msg::Odometry::SharedPtr msg_odom_base_link_;

    // Particle filter state
    double iterationCounter = 0.0;
    bool first_update_ = true;
    bool with_angle_ = true;

    // Last estimated pose
    double last_x_ = 0.0, last_y_ = 0.0, last_theta_ = 0.0;
    double x_last_final = 0.0, y_last_final = 0.0, theta_last_final = 0.0;

    // Color weight lookup
    std::vector<std::pair<double, std::vector<double>>> ColorWeightLookup;

    // Initialization
    void initializeParticles();

    // Particle filter steps
    void motionUpdate(const nav_msgs::msg::Odometry::SharedPtr msg);
    void measurementUpdate(const robot_msgs::msg::FeatureArray::SharedPtr msg);
    void resampleParticles(ResamplingAmount type, ResamplingMethod method);

    // Resampling methods
    void multinomialResample();
    void stratifiedResample();
    void systematicResample();
    void residualResample();

    // Particle management
    void normalizeWeights();
    double maxWeight();
    void replaceWorstParticles(double percentage);
    void injectRandomParticles(double percentage);

    // Pose estimation
    void computeEstimatedPose();
    void publishEstimatedPose();

    // Particle visualization
    void publishParticles();

    // Feature handling
    void storeMapMessage(const robot_msgs::msg::FeatureArray::SharedPtr msg);
    std::vector<map_features::FeatureCorner> getExpectedFeaturesCorner(const Particle &p);
    map_features::FeatureObject getExpectedFeaturesCloserObject(const Particle &p, const std::string type, double x, double y, double z);
    double transformAngleToParticleFrame(double feature_theta_map, double particle_theta);
    double computeAngleLikelihood(double measured_angle, double expected_angle, double sigma);
    DecodedMsg decodeMsg(const robot_msgs::msg::Feature& msg);
    std::vector<geometry_msgs::msg::Point> getKeypointsInNewFrame(
        std::vector<geometry_msgs::msg::Point> keypoints, 
        double  x_base, double y_base, double z_base, double theta_base, 
        double x_new, double y_new, double z_new, double theta_new);
        
    double computeLikelihoodCorner(const Particle &p, double noisy_x, double noisy_y, double noisy_z, double measured_theta, double sigma_pos, double sigma_theta);
    double computeLikelihoodObject(const Particle &p, double noisy_x, double noisy_y, double noisy_z, double measured_theta, double sigma_pos, double sigma_theta, const std::string type, double confidence);

    // Color weight functions
    std::vector<double> colorFromWeight(double weight) const;
    void computeColorWeightLookup();

    // Parameter loading
    void loadParameters();
};

#endif  // PARTICLE_FILTER_HPP