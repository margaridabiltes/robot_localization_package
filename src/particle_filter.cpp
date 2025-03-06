#include "robot_localization_package/particle_filter.hpp"


ParticleFilter::ParticleFilter() : Node("particle_filter"), num_particles_(10000) {
    initializeParticles();

    keypoint_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/keypoints", 10,
        std::bind(&ParticleFilter::measurementUpdate, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilter::motionUpdate, this, std::placeholders::_1) 
    );
    
    
    // Add a TF Buffer and Listener to track odometry
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Add a Publisher for estimated pose
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/estimated_pose", 10
    );

    particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/particles", 10
    );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);


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

    geometry_msgs::msg::TransformStamped odom_tf;
    try {
        odom_tf = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get odometry transform: %s", ex.what());
        return;
    }

    // Extract position and orientation
    double x = odom_tf.transform.translation.x;
    double y = odom_tf.transform.translation.y;
    
    tf2::Quaternion q(
        odom_tf.transform.rotation.x,
        odom_tf.transform.rotation.y,
        odom_tf.transform.rotation.z,
        odom_tf.transform.rotation.w
    );
    double roll, pitch, theta;
    tf2::Matrix3x3(q).getRPY(roll, pitch, theta);

    publishParticles();
    if (first_update_) {
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;
        first_update_ = false; 
        RCLCPP_INFO(this->get_logger(), "Ignored first motion update (initial spawn).");
        return;
    }
    // Only update motion if the robot has moved
    if (std::hypot(x - last_x_, y - last_y_) > 0.01 || std::abs(theta - last_theta_) > 0.01) {
        
        // Compute movement change
        double delta_x = x - last_x_;
        double delta_y = y - last_y_;
        double delta_theta = theta - last_theta_;

        // Save new position for next check
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;

        std::normal_distribution<double> noise_x(0.0, 0.02);
        std::normal_distribution<double> noise_y(0.0, 0.02);
        std::normal_distribution<double> noise_theta(0.0, 0.01);

        for (auto &p : particles_) {
            p.x = std::clamp(p.x + delta_x + noise_x(generator_), -0.75, 0.75);
            p.y = std::clamp(p.y + delta_y + noise_y(generator_), -0.75, 0.75);

            p.theta += delta_theta + noise_theta(generator_);

            // Keep theta within valid range (-π to π)
            if (p.theta > M_PI) p.theta -= 2 * M_PI;
            if (p.theta < -M_PI) p.theta += 2 * M_PI;
        }

        RCLCPP_INFO(this->get_logger(), "Applied motion update.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Robot has not moved. Skipping motion update.");
    }
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
    
        double likelihood = 1.0;
        for(const auto &obs : observed_features){

            double weight_sum = 0.0;
            for (const auto &exp : expected_features) {
                double dist = std::hypot(obs.first - exp.first, obs.second - exp.second);
                weight_sum += std::exp(-dist * dist / (2 * sensor_noise_ * sensor_noise_));
            }
            likelihood *= weight_sum / expected_features.size();  // Normalize likelihood

        }
        p.weight *= likelihood;  
        sum_weights += p.weight;
    }

    // Normalize weights
    for (auto &p : particles_) {
        p.weight /= sum_weights; 
    }


    resampleParticles();

    //publishParticles();

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
    std::uniform_real_distribution<double> random_noise(-0.05, 0.05);

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

        // Add some noise to prevent premature collapse
        sampled.x += random_noise(generator_);
        sampled.y += random_noise(generator_);
        sampled.theta += random_noise(generator_);

        new_particles.push_back(sampled);
    }

    particles_ = new_particles;

    RCLCPP_INFO(this->get_logger(), "Resampled particles.");


    //publishEstimatedPose();
    
}



void ParticleFilter::publishEstimatedPose() {
    if (particles_.empty()) return;

    // Compute weighted average pose
    double x_sum = 0, y_sum = 0, theta_sum = 0;
    for (const auto &p : particles_) {
        x_sum += p.x * p.weight;
        y_sum += p.y * p.weight;
        theta_sum += p.theta * p.weight;
    }

    double x_est = x_sum;
    double y_est = y_sum;
    double theta_est = theta_sum;

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = x_est;
    pose_msg.pose.position.y = y_est;

    // Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_est);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_->publish(pose_msg);

    // Publish `map -> odom` transform
    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = this->get_clock()->now();
    map_to_odom_tf.header.frame_id = "map";
    map_to_odom_tf.child_frame_id = "odom";

    map_to_odom_tf.transform.translation.x = x_est;
    map_to_odom_tf.transform.translation.y = y_est;
    map_to_odom_tf.transform.translation.z = 0.0;

    map_to_odom_tf.transform.rotation.x = q.x();
    map_to_odom_tf.transform.rotation.y = q.y();
    map_to_odom_tf.transform.rotation.z = q.z();
    map_to_odom_tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(map_to_odom_tf);

    RCLCPP_INFO(this->get_logger(), "Published estimated pose and map->odom transform.");
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
    auto node = std::make_shared<ParticleFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

