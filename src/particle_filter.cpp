#include "robot_localization_package/particle_filter.hpp"


ParticleFilter::ParticleFilter() : Node("particle_filter"), num_particles_(10000),
    last_x_(0.0), last_y_(0.0), last_theta_(0.0), first_update_(true),
    msg_odom_base_link_(nullptr), last_keypoint_msg_(nullptr)
{
    std::cout << "ParticleFilter Constructor START" << std::endl;  
    RCLCPP_INFO(this->get_logger(), "Initializing particle filter node.");

    initializeParticles();

    keypoint_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/keypoints", 10, 
        std::bind(&ParticleFilter::storeKeypointMessage, this, std::placeholders::_1)  
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilter::motionUpdate, this, std::placeholders::_1) 
    );

    odom_sub_2 = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilter::storeOdomBaseLink, this, std::placeholders::_1) 
    );

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_pose_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ParticleFilter::publishEstimatedPose, this));

    RCLCPP_INFO(this->get_logger(), "Particle filter node initialized successfully.");
}


void ParticleFilter::storeOdomBaseLink(const nav_msgs::msg::Odometry::SharedPtr msg){
    msg_odom_base_link_ = msg;
}

void ParticleFilter::storeKeypointMessage(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    last_keypoint_msg_ = msg;  
}

void ParticleFilter::initializeParticles() {
    std::uniform_real_distribution<double> dist_x(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_y(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_theta(0, 2 * M_PI); 

    particles_.resize(num_particles_);
    for (auto &p : particles_) {
        p.x = dist_x(generator_);
        p.y = dist_y(generator_);
        p.theta = dist_theta(generator_);
        p.init_x = p.x;
        p.init_y = p.y;
        p.init_theta = p.theta;
        p.weight = 1.0 / num_particles_;
    }

    RCLCPP_INFO(this->get_logger(), "Initialized %d particles across map.", num_particles_);
}

std::vector<std::pair<double, double>> ParticleFilter::getExpectedFeatures(const Particle &p) {
    std::vector<std::pair<double, double>> features_map = {
        {-0.75, 0.75}, {0.75, 0.75}, {0.75, -0.75}, {-0.75, -0.75}
    };

    std::vector<std::pair<double, double>> features_particle;
    for (const auto &f : features_map) {
        double dx = f.first - p.x;  
        double dy = f.second - p.y;

        double x = std::cos(p.theta) * dx + std::sin(p.theta) * dy;
        double y = -std::sin(p.theta) * dx + std::cos(p.theta) * dy;

        features_particle.emplace_back(x, y);
    }
    return features_particle;
}

void ParticleFilter::motionUpdate(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (particles_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No particles to update.");
        return;
    }

    double odom_x = msg->pose.pose.position.x;
    double odom_y = msg->pose.pose.position.y;

    tf2::Quaternion odom_q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, odom_theta;
    tf2::Matrix3x3(odom_q).getRPY(roll, pitch, odom_theta);

    std::normal_distribution<double> noise_x(0.0, 0.05);
    std::normal_distribution<double> noise_y(0.0, 0.05);
    std::normal_distribution<double> noise_theta(0.0, 0.01);

    if (first_update_) {
        last_x_ = odom_x;
        last_y_ = odom_y;
        last_theta_ = odom_theta;
        first_update_ = false; 
        RCLCPP_INFO(this->get_logger(), "Ignored first motion update (initial spawn).");
        return;
    }

    double delta_x = odom_x - last_x_;
    double delta_y = odom_y - last_y_;
    double delta_theta = odom_theta - last_theta_;
    double delta_distance = std::hypot(delta_x, delta_y);

    if (delta_distance > 0.01 || std::abs(delta_theta) > 0.1) {

        if (!last_keypoint_msg_) {
            RCLCPP_WARN(this->get_logger(), "No keypoint message available yet.");
            return;
        }

        for (auto &p : particles_) {
            //Save new position for next check
            last_x_ = odom_x;
            last_y_ = odom_y;
            last_theta_ = odom_theta;

            p.x = p.init_x + odom_x * std::cos(p.init_theta) - odom_y * std::sin(p.init_theta) + noise_x(generator_);
            p.y = p.init_y + odom_x * std::sin(p.init_theta) + odom_y * std::cos(p.init_theta) + noise_y(generator_);
            p.theta = p.init_theta + odom_theta + noise_theta(generator_);

            //  Normalize theta to [-π, π]
            if (p.theta > M_PI) p.theta -= 2 * M_PI;
            if (p.theta < -M_PI) p.theta += 2 * M_PI;
        }
        RCLCPP_INFO(this->get_logger(), "Applied motion update.");
        
        measurementUpdate(last_keypoint_msg_);
    }

    publishParticles();
}

void ParticleFilter::measurementUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  
    if (particles_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No particles to update.");
        return;
    }

    // Extract observed features from the PointCloud2 message
    std::vector<std::pair<double, double>> observed_features;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        observed_features.emplace_back(*iter_x, *iter_y);
    }

    double sum_weights = 0;

    // Update particle weights based on feature matching
    int i = 0;
    for (auto &p : particles_) {
        i++;
        std::vector<std::pair<double, double>> expected_features = getExpectedFeatures(p);
    
        double likelihood = 0.0;  
        for (const auto &obs : observed_features) {
            double min_dist = std::numeric_limits<double>::max();
            for (const auto &exp : expected_features) {
                double dist = std::hypot(obs.first - exp.first, obs.second - exp.second);
                min_dist = std::min(min_dist, dist);
            }
            likelihood += std::exp(-min_dist / sensor_noise_); 
        }

        p.weight *= likelihood;  
        sum_weights += p.weight;
    }

    // Normalize weights
    for (auto &p : particles_) {
        p.weight /= sum_weights; 
    }
    double effective_sample_size = 0.0;
    for (const auto &p : particles_) {
        effective_sample_size += p.weight * p.weight;
    }
    effective_sample_size = 1.0 / effective_sample_size;
    
    if (effective_sample_size < num_particles_ * 0.5) {  // Only resample if particles collapse
        resampleParticles();
    } else {
        RCLCPP_INFO(this->get_logger(), "Skipping resampling, particles are well-distributed.");
    }
}

void ParticleFilter::resampleParticles() {
    if (particles_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No particles to resample.");
        return;
    }

    // Compute the cumulative sum of weights
    std::vector<double> cumulative_weights;
    double sum_weights = 0.0;
    for (const auto &p : particles_) {
        sum_weights += p.weight;
        cumulative_weights.push_back(sum_weights);
    }

    // Check for zero weights (edge case)
    if (sum_weights == 0.0) {
        RCLCPP_WARN(this->get_logger(), "All particle weights are zero. Reinitializing particles.");
        initializeParticles();
        return;
    }

    std::vector<Particle> new_particles;
    std::normal_distribution<double> noise_x(0.0, 0.05);  
    std::normal_distribution<double> noise_y(0.0, 0.05);
    std::normal_distribution<double> noise_theta(0.0, 0.01);  


    // Resampling wheel algorithm
    std::uniform_real_distribution<double> dist(0.0, sum_weights / num_particles_);
    double r = dist(generator_);
    int index = 0;
    double step = sum_weights / num_particles_;

    for (int i = 0; i < num_particles_; i++) {
        double target = r + i * step;

        // Ensure index remains within bounds
        while (index < static_cast<int>(cumulative_weights.size()) - 1 && cumulative_weights[index] < target) {
            index++;
        }

        index = std::min(index, static_cast<int>(particles_.size()) - 1);  // Prevent out-of-bounds access

        Particle sampled = particles_[index];  // Now `index` is properly assigned

        if (std::abs(sampled.x - particles_[index].x) < 0.01) {
            sampled.x += noise_x(generator_);
            sampled.y += noise_y(generator_);
        }
        sampled.theta += noise_theta(generator_);

        // Normalize theta to [-π, π]
        if (sampled.theta > M_PI) sampled.theta -= 2 * M_PI;
        if (sampled.theta < -M_PI) sampled.theta += 2 * M_PI;

        new_particles.push_back(sampled);
    }

    particles_ = new_particles;

    RCLCPP_INFO(this->get_logger(), "Resampled particles.");


    computeEstimatedPose();
    
}

void ParticleFilter::computeEstimatedPose(){
    if (particles_.empty()) return;

    // Sort particles by weight (highest first)
    std::vector<Particle> sorted_particles = particles_;
    std::sort(sorted_particles.begin(), sorted_particles.end(), 
        [](const Particle &a, const Particle &b) {
            return a.weight > b.weight;  // Sort in descending order
        });

    // Use only the top 10 particles
    int num_top_particles = std::min(10, static_cast<int>(sorted_particles.size()));

    double x_sum = 0, y_sum = 0, theta_sum = 0, weight_sum = 0;

    for (int i = 0; i < num_top_particles; i++) {
        const auto &p = sorted_particles[i];
        x_sum += p.x * p.weight;
        y_sum += p.y * p.weight;
        theta_sum += p.theta * p.weight;
        weight_sum += p.weight;
    }

    // Normalize weights
    if (weight_sum > 0) {
        x_sum /= weight_sum;
        y_sum /= weight_sum;
        theta_sum /= weight_sum;
    }

    x_last_final=x_sum;
    y_last_final=y_sum;
    theta_last_final=theta_sum;

    if (theta_last_final > M_PI) theta_last_final -= 2 * M_PI;
    if (theta_last_final < -M_PI) theta_last_final += 2 * M_PI;
    
}
void ParticleFilter::publishEstimatedPose() {
    if (particles_.empty()) return;

    std::cout << "Publishing Estimated Pose" << std::endl;
    
    if (!msg_odom_base_link_) {
        RCLCPP_WARN(this->get_logger(), "Skipping pose publication: No odometry data available.");
        return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = x_last_final;
    pose_msg.pose.position.y = y_last_final;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_last_final);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);

    // Publish `map -> base_link` transform
    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = this->get_clock()->now();
    map_to_odom_tf.header.frame_id = "map";
    map_to_odom_tf.child_frame_id = "odom";

    double odom_x = msg_odom_base_link_->pose.pose.position.x;
    double odom_y = msg_odom_base_link_->pose.pose.position.y;

    tf2::Quaternion odom_base_link_q(
        msg_odom_base_link_->pose.pose.orientation.x,
        msg_odom_base_link_->pose.pose.orientation.y,
        msg_odom_base_link_->pose.pose.orientation.z,
        msg_odom_base_link_->pose.pose.orientation.w
    );

    // Calculate map -> odom transform
    map_to_odom_tf.transform.translation.x = x_last_final - odom_x;
    map_to_odom_tf.transform.translation.y = y_last_final - odom_y;
    map_to_odom_tf.transform.translation.z = 0.0;

    tf2::Quaternion q_map, q_odom, q_correction;
    q_map.setRPY(0, 0, theta_last_final);
    q_odom.setX(odom_base_link_q.x());
    q_odom.setY(odom_base_link_q.y());
    q_odom.setZ(odom_base_link_q.z());
    q_odom.setW(odom_base_link_q.w());

    q_correction = q_map * q_odom.inverse();
    map_to_odom_tf.transform.rotation.x = q_correction.x();
    map_to_odom_tf.transform.rotation.y = q_correction.y();
    map_to_odom_tf.transform.rotation.z = q_correction.z();
    map_to_odom_tf.transform.rotation.w = q_correction.w();

    tf_broadcaster_->sendTransform(map_to_odom_tf);

    RCLCPP_INFO(this->get_logger(), "Published estimated pose (Top 10 weighted particles).");
}

void ParticleFilter::publishParticles() {
    if (particles_.empty()) return;

    geometry_msgs::msg::PoseArray particles_msg;
    particles_msg.header.stamp = this->get_clock()->now();
    particles_msg.header.frame_id = "map";

    for (const auto &p : particles_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, p.theta);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        particles_msg.poses.push_back(pose);
    }

    particles_pub_->publish(particles_msg);
}


// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilter>());
    rclcpp::shutdown();
    return 0;
}

