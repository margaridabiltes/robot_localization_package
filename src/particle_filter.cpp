#include "robot_localization_package/particle_filter.hpp"


ParticleFilter::ParticleFilter() : Node("particle_filter"), num_particles_(1000),
    last_x_(0.0), last_y_(0.0), last_theta_(0.0), iterationCounter(0.0), first_update_(true),
    msg_odom_base_link_(nullptr), last_map_msg_(nullptr)
{
    initializeFeatures();

    std::cout << "ParticleFilter Constructor START" << std::endl;  
    RCLCPP_INFO(this->get_logger(), "Initializing particle filter node.");

    feature_sub_ = this->create_subscription<robot_msgs::msg::FeatureArray>(
        "/corner", 10,
        std::bind(&ParticleFilter::storeMapMessage, this, std::placeholders::_1)
    );    

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilter::motionUpdate, this, std::placeholders::_1) 
    );

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    particles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //timer_pose_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ParticleFilter::publishEstimatedPose, this));

    while (rclcpp::ok() && !last_map_msg_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for the first keypoint message...");
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    initializeParticles();
    computeColorWeightLookup();

    RCLCPP_INFO(this->get_logger(), "Particle filter node initialized successfully.");
}

//! auxiliar functions start!//

#pragma region auxiliar functions
void ParticleFilter::initializeFeatures(){
    corner_features.emplace_back(-0.75, 0.75, -45*M_PI/180); 
    corner_features.emplace_back(0.75, 0.75, -135*M_PI/180);          
    corner_features.emplace_back(0.75, -0.75, 135*M_PI/180); 
    corner_features.emplace_back(-0.75, -0.75, 45*M_PI/180); 
    
  /*   square_features.emplace_back(-0.265062, 0.1,0,0);
    square_features.emplace_back(-0.115895, 0.1,0,0);
    square_features.emplace_back(0.44,0.1,0,0);
    square_features.emplace_back(0.29726, 0.1,0,0);
    square_features.emplace_back(-0.158467,0.1,0,0); */
}

void ParticleFilter::normalizeWeights(){
    double sum_weights = 0;
    for (auto &p : particles_) {
        sum_weights += p.weight;
    }

    for (auto &p : particles_) {
        p.weight /= sum_weights;
    }
}

double ParticleFilter::maxWeight(){
    double max_weight = 0.0;
    for (const auto &p : particles_) {
        max_weight = std::max(max_weight, p.weight);
    }
    return max_weight;
}

void ParticleFilter::computeColorWeightLookup() {
    double average_weight = 1.0 / num_particles_;

    ColorWeightLookup = {
        {0.9 * average_weight, {1.0, 1.0, 1.0}}, // White
        {average_weight, {0.56, 0.0, 1.0}}, // Violet
        {1.05 * average_weight, {0.0, 0.0, 1.0}}, // Blue
        {1.1 * average_weight, {0.0, 1.0, 0.5}}, // Cyan
        {1.15 * average_weight, {0.0, 1.0, 0.0}}, // Green
        {1.2 * average_weight, {1.0, 1.0, 0.0}}, // Yellow
        {1.5 * average_weight, {1.0, 0.5, 0.0}}, // Orange
        {std::numeric_limits<double>::max(), {1.0, 0.0, 0.0}}  // Red
    };
}

std::vector<double> ParticleFilter::colorFromWeight(double weight) {

    for (const auto& entry : ColorWeightLookup) {
        if (weight < entry.first) {
            return entry.second; // Return the corresponding RGB color
        }
    }
    return {1.0, 0.0, 0.0}; // Default to Red (should never reach here)
}

void ParticleFilter::publishParticles() {
    if (particles_.empty()) return;
    
    //geometry_msgs::msg::PoseArray particles_msg;
    visualization_msgs::msg::MarkerArray marker_array;

    double max_weight = maxWeight();
    //std::cout << "max_weight no publish: " << max_weight << std::endl;

    int i = 0;
    for (const auto &p : particles_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "particles";
        marker.id = i++;

        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, p.theta);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.07;
        marker.scale.y = 0.005;
        marker.scale.z = 0.01;
        
        std::vector<double> color = colorFromWeight(p.weight);
        marker.color.a = 1.0;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

        marker_array.markers.push_back(marker);
    }

    particles_pub_->publish(marker_array);
}

void ParticleFilter::replaceWorstParticles( double percentage ) {
    std::sort(particles_.begin(), particles_.end(), 
              [](const Particle &a, const Particle &b) { return a.weight < b.weight; });

    int num_replace = num_particles_ * percentage;

    std::uniform_real_distribution<double> dist_x(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_y(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    for (int i = 0; i < num_replace; i++) {
        particles_[i].x = dist_x(generator_);
        particles_[i].y = dist_y(generator_);
        particles_[i].theta = dist_theta(generator_);
        particles_[i].weight = 1.0 / num_particles_; 
    }

    RCLCPP_INFO(this->get_logger(), "Replaced %d worst particles with random ones.", num_replace);

    normalizeWeights();
}

void ParticleFilter::injectRandomParticles(double percentage){
    //replace random particles
    std::uniform_real_distribution<double> dist_x(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_y(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    int num_replace = num_particles_ * percentage;

    //get random num_replace particles index
    std::vector<int> random_index;
    for(int i = 0; i < num_replace; i++){
        random_index.push_back(rand() % (int)num_particles_);
    }

    for (int i = 0; i < num_replace; i++) {
        particles_[random_index[i]].x = dist_x(generator_);
        particles_[random_index[i]].y = dist_y(generator_);
        particles_[random_index[i]].theta = dist_theta(generator_);
        particles_[random_index[i]].weight = 1.0 / num_particles_; 
    }

}

void ParticleFilter::storeMapMessage(const robot_msgs::msg::FeatureArray::SharedPtr msg) {
    last_map_msg_ = msg;  
}

std::vector<ParticleFilter::FeatureCorner> ParticleFilter::getExpectedFeatures(const Particle &p) {

    std::vector<FeatureCorner> features_particle;

    double cos_theta = std::cos(p.theta);
    double sin_theta = std::sin(p.theta);

    
    for(const auto &f: corner_features){
        double map_x = f.x;
        double map_y = f.y;
        double corner_theta = f.theta; 

        double particle_x = cos_theta * (map_x - p.x) + sin_theta * (map_y - p.y);
        double particle_y = -sin_theta * (map_x - p.x) + cos_theta * (map_y - p.y);

        features_particle.emplace_back(particle_x, particle_y, corner_theta);

    }
    
    return features_particle;
}

double ParticleFilter::transformAngleToParticleFrame(double feature_theta_map, double particle_theta) {
    double angle = feature_theta_map - particle_theta;

    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;

    return angle;
}

double ParticleFilter::computeAngleLikelihood(double measured_angle, double expected_angle, double sigma) {

    double error = measured_angle - expected_angle;

    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    double coeff = 1.0 / std::sqrt(2.0 * M_PI * sigma * sigma);
    double exponent = -0.5 * (error * error) / (sigma * sigma);

    return coeff * std::exp(exponent);
}

ParticleFilter::DecodedMsg ParticleFilter::decodeMsg(const robot_msgs::msg::Feature& msg){
    DecodedMsg feature;
    
    feature.x = msg.position.x;
    feature.y = msg.position.y;
    feature.type = msg.type;

    tf2::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    feature.theta = yaw; 

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            feature.covariance_pos[i][j] = msg.position_covariance[i * 3 + j];
            feature.covariance_angle[i][j] = msg.orientation_covariance[i * 3 + j];
        }
    }

    return feature;
}


#pragma endregion auxiliar functions

//! auxiliar functions end!//

//! Resampling functions start !//

#pragma region resampling functions

void ParticleFilter::multinomialResample() {
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    // Compute cumulative weights
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < num_particles_; i++) {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles_[i].weight;
    }

    std::uniform_real_distribution<double> dist(0.0, cumulative_weights.back());
    for (size_t i = 0; i < num_particles_; i++) {
        double r = dist(generator_);
        auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), r);
        int index = std::distance(cumulative_weights.begin(), it);
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;

}

void ParticleFilter::stratifiedResample() {
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    // Compute cumulative weights
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < num_particles_; i++) {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles_[i].weight;
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    double r = dist(generator_);

    int index = 0;
    for (size_t i = 0; i < num_particles_; i++) {
        double U = r + (i / static_cast<double>(num_particles_));
        while (U > cumulative_weights[index]) index++;
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;
}

void ParticleFilter::systematicResample() {
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    // Compute cumulative weights
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < num_particles_; i++) {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles_[i].weight;
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    double r = dist(generator_);

    int index = 0;
    for (size_t i = 0; i < num_particles_; i++) {
        double U = r + (i / static_cast<double>(num_particles_));
        while (U > cumulative_weights[index]) index++;
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;
}

void ParticleFilter::residualResample() {
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    std::vector<double> residual_weights;
    int total_copies = 0;

    for (const auto &p : particles_) {
        int num_copies = static_cast<int>(p.weight * num_particles_);
        total_copies += num_copies;
        for (int j = 0; j < num_copies; j++) new_particles.push_back(p);
        residual_weights.push_back((p.weight * num_particles_) - num_copies);
    }

    std::vector<double> cumulative_weights;
    double sum_residuals = 0.0;
    for (double rw : residual_weights) {
        sum_residuals += rw;
        cumulative_weights.push_back(sum_residuals);
    }

    std::uniform_real_distribution<double> dist(0.0, sum_residuals);
    while (total_copies < num_particles_) {
        double r = dist(generator_);
        for (size_t i = 0; i < cumulative_weights.size(); i++) {
            if (r <= cumulative_weights[i]) {
                new_particles.push_back(particles_[i]);
                total_copies++;
                break;
            }
        }
    }

    particles_ = new_particles;
}

#pragma endregion resampling functions

//! Resampling functions end !//

//! Particle Filter Functions !//

#pragma region pf functions

void ParticleFilter::initializeParticles() {

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);

    std::uniform_real_distribution<double> dist_x(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_y(-0.75, 0.75);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI); 

    log_file_.open("log_pf.txt", std::ios::app);
    if (!log_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open log file for writing.");
        return;
    }
    particles_.resize(num_particles_);
    for (auto &p : particles_) {
        p.x = dist_x(generator_);
        p.y = dist_y(generator_);
        p.theta = dist_theta(generator_);
        p.weight = 1.0 / num_particles_;
    } 

    RCLCPP_INFO(this->get_logger(), "Initialized %f particles", num_particles_);
}

void ParticleFilter::motionUpdate(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (particles_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No particles to update.");
        return;
    }

    msg_odom_base_link_ = msg;

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

    std::uniform_real_distribution<double> noise_x(-noise_x_, noise_x_);
    std::uniform_real_distribution<double> noise_y(-noise_y_, noise_y_);
    std::uniform_real_distribution<double> noise_theta(-noise_theta_, noise_theta_);
    
    double delta_x_odom = odom_x - last_x_;
    double delta_y_odom = odom_y - last_y_;
    double delta_distance = std::hypot(delta_x_odom, delta_y_odom);
    
    double alpha_odom = atan2(delta_y_odom, delta_x_odom);
    double alpha_robot = alpha_odom - last_theta_;
    double delta_x_robot = delta_distance * std::cos(alpha_robot);
    double delta_y_robot = delta_distance * std::sin(alpha_robot);
    
    double delta_theta_odom = odom_theta - last_theta_;

    if (delta_distance > 0.2 || std::abs(delta_theta_odom) > 0.2) {
        if (!last_map_msg_) {
            RCLCPP_WARN(this->get_logger(), "No keypoint message available yet.");
            return;
        }

        for(auto &p : particles_){

            p.x += delta_x_robot* std::cos(p.theta) - delta_y_robot * std::sin(p.theta) + noise_x(generator_)  ;
            p.y += delta_x_robot* std::sin(p.theta) + delta_y_robot * std::cos(p.theta)  + noise_y(generator_) ;
            p.theta +=  delta_theta_odom  + noise_theta(generator_) ;
    
            if (p.theta > M_PI) p.theta -= 2 * M_PI;
            if (p.theta < -M_PI) p.theta += 2 * M_PI;
        }

        last_x_ = odom_x;
        last_y_ = odom_y;
        last_theta_ = odom_theta;
        
        measurementUpdate(last_map_msg_);
    }

    publishParticles();
}

void ParticleFilter::measurementUpdate(const robot_msgs::msg::FeatureArray::SharedPtr msg){
    if (particles_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No particles to update.");
        return;
    }

    for (auto &p : particles_) {
        std::vector<FeatureCorner> expected_features = getExpectedFeatures(p);
        double likelihood = 0;  

        for (const auto &obs_msg : msg->features) {

            DecodedMsg obs = decodeMsg(obs_msg);

            double distance = std::hypot(obs.x, obs.y);

            double sigma_x = std::sqrt(obs.covariance_pos[0][0]);  
            double sigma_y = std::sqrt(obs.covariance_pos[1][1]); 
            double sigma_theta = std::sqrt(obs.covariance_angle[2][2]);  
            double sigma_pos = std::sqrt((sigma_x * sigma_x + sigma_y * sigma_y) / 2.0);

            std::normal_distribution<double> measurement_noise(0, sigma_pos);
            std::normal_distribution<double> noise_pos_x(0.0, sigma_x);
            std::normal_distribution<double> noise_pos_y(0.0, sigma_y);
            std::normal_distribution<double> noise_theta(0.0, sigma_theta);
            
            double measured_theta = obs.theta + noise_theta(generator_);


            double min_dist = std::numeric_limits<double>::max();
            
            double noisy_x = obs.x + noise_pos_x(generator_);
            double noisy_y = obs.y + noise_pos_y(generator_);

            FeatureCorner best_corner(0, 0, 0);
    
            for (const auto &exp : expected_features) {
                double dist = std::hypot(noisy_x - exp.x, noisy_y - exp.y);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_corner = exp; 
                }
            }

            double expected_feature_angle = transformAngleToParticleFrame(best_corner.theta, p.theta);

            double angle_likelihood = computeAngleLikelihood(measured_theta, expected_feature_angle, sigma_theta);

            double distance_likelihood=(std::exp(- (min_dist * min_dist) / (2 * sigma_pos * sigma_pos)))/std::sqrt(2 * M_PI * sigma_pos * sigma_pos);

            if(with_angle_){
                likelihood += (angle_likelihood + distance_likelihood);
            }
            else{
                likelihood += distance_likelihood;
            }
        }
    
        p.weight *= likelihood;     
    }
    
    for ( auto &p : particles_){
        if(p.x > 0.75 || p.x < -0.75 || p.y > 0.75 || p.y < -0.75){
            p.weight =p.weight/ 2;
        }
    }   
  
    normalizeWeights();


    resampleParticles(ResamplingAmount::MAX_WEIGHT, ResamplingMethod::RESIDUAL); 
    
    if(resample_flag_==false){
        replaceWorstParticles(0.05);

    }
    else{
        resample_flag_=false;
    }

}

void ParticleFilter::resampleParticles(ResamplingAmount type, ResamplingMethod method) {
    if (particles_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No particles to resample.");
        return;
    }

    std::cout<<"Max weight: "<<maxWeight()<<std::endl;
    log_file_<<"Max weight: "<<maxWeight()<<"\n";

    double ess = 1.0 / std::accumulate(particles_.begin(), particles_.end(), 0.0,
    [](double sum, const Particle &p) { return sum + (p.weight * p.weight); });

    std::cout<<"ESS: "<<ess<<std::endl;

    double max_weight = maxWeight();

    switch(type){
        case ResamplingAmount::ESS:
            if (ess > num_particles_ * 0.5) {
                RCLCPP_INFO(this->get_logger(), "Skipping resampling, particles are well-distributed.");
                return;
            }
            break;
        case ResamplingAmount::MAX_WEIGHT:
            if (max_weight < 4/num_particles_) {
                RCLCPP_INFO(this->get_logger(), "Skipping resampling, max weight is high.");
                return;
            }
            break;
    }

    resample_flag_=true;

    switch (method) {
        case ResamplingMethod::MULTINOMIAL:
            multinomialResample();
            break;
        case ResamplingMethod::STRATIFIED:
            stratifiedResample();
            break;
        case ResamplingMethod::SYSTEMATIC:
            systematicResample();
            break;
        case ResamplingMethod::RESIDUAL:
            residualResample();
            break;
    }

    for(auto &p : particles_){
        p.weight = 1.0 / num_particles_;
    }

    iterationCounter++;

    if(iterationCounter == MAX_ITERATION){
        std::cout<<"Injecting particles"<<std::endl;
        injectRandomParticles(0.2);
        iterationCounter=0;
    } 

}

void ParticleFilter::computeEstimatedPose(){
    if (particles_.empty()) return;

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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilter>());
    rclcpp::shutdown();
    return 0;
}

#pragma endregion pf functions

//! Particle Filter Functions !//