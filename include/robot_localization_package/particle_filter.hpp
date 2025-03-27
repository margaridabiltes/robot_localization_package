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
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include "robot_msgs/msg/feature.hpp"
#include "robot_msgs/msg/feature_array.hpp"
#include "robot_localization_package/FeatureStruct.hpp"
#include "robot_localization_package/MapLoader.hpp"
#include <vector>
#include <random>
#include <fstream>


#define noise_x_ 0.05
#define noise_y_ 0.05
#define noise_theta_ 0.1
#define MAX_ITERATION 20


class ParticleFilter : public rclcpp::Node {
public:
    ParticleFilter();

private:
    struct Particle {
        double x, y, theta; 
        double weight;      
    };

    struct DecodedMsg {
        double x;
        double y;
        double theta;  
        std::string type;
        std::array<std::array<double, 3>, 3> covariance_pos;
        std::array<std::array<double, 3>, 3> covariance_angle;
    };
    
    enum class ResamplingMethod {
        MULTINOMIAL,
        STRATIFIED,
        SYSTEMATIC,
        RESIDUAL
    };

    enum class ResamplingAmount {
        ESS,
        MAX_WEIGHT
    };

    
    std::default_random_engine generator_;
    double sensor_noise_ = 0.5; 
    double angle_sigma_ = M_PI / 36.0;

    // Feature storage
    map_features::MapLoader map_loader_;

    std::vector<map_features::FeaturePtr> global_features_;

    std::string map_features_;

    bool new_map=false;
    
    //pf
    std::ofstream log_file_;

    double num_particles_; 
    std::vector<Particle> particles_;

    bool resample_flag_ = false;

    void initializeParticles();
    void motionUpdate(const nav_msgs::msg::Odometry::SharedPtr msg);
    void measurementUpdate(const robot_msgs::msg::FeatureArray::SharedPtr msg);

    double computeLikelihoodCorner(const Particle &p, double noisy_x, double noisy_y, double noisy_z, double measured_theta, double sigma_pos, double sigma_theta);
    double computeLikelihoodObject(const Particle &p, double noisy_x, double noisy_y, double noisy_z, double measured_theta, double sigma_pos, double sigma_theta, const std::string type);

    void resampleParticles(ResamplingAmount type,ResamplingMethod method);
    void computeEstimatedPose();
    void publishEstimatedPose();
   
    void publishParticles();
    void replaceWorstParticles(double percentage);
    void injectRandomParticles(double percentage);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr particles_pub_;
    rclcpp::Subscription<robot_msgs::msg::FeatureArray>::SharedPtr feature_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_pose_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    robot_msgs::msg::FeatureArray::SharedPtr last_map_msg_; 
    nav_msgs::msg::Odometry::SharedPtr msg_odom_base_link_;


    //resample
    void multinomialResample();
    void stratifiedResample();
    void systematicResample();
    void residualResample();
    
    //aux    
    void normalizeWeights();
    double maxWeight();
    void storeMapMessage(const robot_msgs::msg::FeatureArray::SharedPtr msg);
    std::vector<map_features::FeatureCorner> getExpectedFeaturesCorner(const Particle &p);
    map_features::FeatureObject getExpectedFeaturesCloserObject(const Particle &p, const std::string type, double x, double y, double z);
    double transformAngleToParticleFrame(double feature_theta_map, double particle_theta);
    double computeAngleLikelihood(double measured_angle, double expected_angle, double sigma);
    ParticleFilter::DecodedMsg decodeMsg(const robot_msgs::msg::Feature& msg);

    double iterationCounter;
    bool first_update_ = true;  

    bool with_angle_ = true;

    double last_x_, last_y_, last_theta_;
    double x_last_final = 0.0;
    double y_last_final = 0.0;
    double theta_last_final = 0.0;

    //aux color weight
    std::vector<double> colorFromWeight(double weight);
    void computeColorWeightLookup();
    std::vector<std::pair<double, std::vector<double>>> ColorWeightLookup;

};
#endif