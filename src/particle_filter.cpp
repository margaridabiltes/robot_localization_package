#include "robot_localization_package/particle_filter.hpp"

struct PGMImage2
{
    int width;
    int height;
    int max_val;
    std::vector<uint8_t> data;

    void load(const std::string &path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open())
            throw std::runtime_error("Could not open PGM file");

        std::string line;
        std::getline(file, line);
        if (line != "P5")
            throw std::runtime_error("Only binary PGM (P5) supported");

        do
        {
            std::getline(file, line);
        } while (line[0] == '#');

        std::stringstream ss(line);
        ss >> width >> height;

        file >> max_val;
        file.get(); // consume the newline

        data.resize(width * height);
        file.read(reinterpret_cast<char *>(data.data()), data.size());
    }

    uint8_t pixel(int x, int y) const
    {
        return data[y * width + x];
    }
};

std::tuple<double, double> pixelToWorld(int x_pix, int y_pix, double resolution, const std::vector<double> &origin, int image_height)
{
    double x = origin[0] + (x_pix + 0.5) * resolution;
    double y = origin[1] + (image_height - y_pix - 0.5) * resolution;
    return {x, y};
}

std::tuple<int, int> worldToPixel(double x_world, double y_world, double resolution, const std::vector<double> &origin, int image_height)
{
    int x_pix = static_cast<int>((x_world - origin[0]) / resolution - 0.5);
    int y_pix = image_height - 1 - static_cast<int>((y_world - origin[1]) / resolution - 0.5);
    return {x_pix, y_pix};
}

// Check if particle is in white part of pgm
bool isParticleInFreeSpace(double x_world, double y_world, const PGMImage &pgm, double resolution, const std::vector<double> &origin)
{
    // Convert world to pixel coordinates
    int x_pix = static_cast<int>((x_world - origin[0]) / resolution);
    int y_pix = pgm.height - 1 - static_cast<int>((y_world - origin[1]) / resolution); // y inverted

    // Check bounds
    if (x_pix < 0 || x_pix >= pgm.width || y_pix < 0 || y_pix >= pgm.height)
    {
        return false; // out of bounds = not free
    }

    uint8_t val = pgm.pixel(x_pix, y_pix);

    // Interpret pixel value
    return val >= 254; // white = free
}

ParticleFilter::ParticleFilter() : Node("particle_filter"),
                                   last_x_(0.0), last_y_(0.0), last_theta_(0.0), iterationCounter(0.0), first_update_(true),
                                   msg_odom_base_link_(nullptr), last_map_msg_(nullptr)
{
    std::cout << "ParticleFilter Constructor START" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Initializing particle filter node.");

    // Load parameters from the parameter file
    loadParameters();

    // Retrieve the map_features parameter passed from the launch file

    this->get_parameter("map_features", map_features_);

    if (map_features_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No map features provided. Please set the 'map_features' parameter.");
        return;
    }

    // Load the map features from the YAML file and store them in the global map
    map_loader_.loadToGlobalMap(map_features_);
    global_features_ = map_loader_.getGlobalFeatureMap();

    // Subscribe to the features observed by the robot and odometry topics
    feature_sub_ = this->create_subscription<robot_msgs::msg::FeatureArray>(
        "/features", 10,
        std::bind(&ParticleFilter::storeMapMessage, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilter::motionUpdate, this, std::placeholders::_1));

    // Create publishers for the estimated pose and particles
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/estimated_pose", 10);
    particles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/particles", 10);

    // Create a transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create a timer to publish the estimated pose and particles
    timer_pose_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&ParticleFilter::publishEstimatedPose, this));

    // Initialize the color pallete for the particles weights
    computeColorWeightLookup();

    // Load PGM and calculate free space
    calculateFreeSpaceFromPGM();

    // Initialize the particles
    initializeParticles_pgm();

    while (rclcpp::ok() && !last_map_msg_)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for the first keypoint message...");
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(), "Particle filter node initialized successfully.");
}

//! auxiliar functions start!//

#pragma region auxiliar functions

// load the paramaters for the node
void ParticleFilter::loadParameters()
{
    RCLCPP_INFO(this->get_logger(), "Loading particle filter parameters...");

    bool success = true;

    this->declare_parameter("num_particles", NUM_PARTICLES);
    this->declare_parameter("motion_delta_distance", MOTION_DELTA_DISTANCE);
    this->declare_parameter("motion_delta_angle", MOTION_DELTA_ANGLE);
    this->declare_parameter("motion_x_variance", MOTION_X_VARIANCE);
    this->declare_parameter("motion_y_variance", MOTION_Y_VARIANCE);
    this->declare_parameter("motion_angle_variance", MOTION_ANGLE_VARIANCE);
    this->declare_parameter("resample_ess_threshold", RESAMPLE_ESS_THRESHOLD);
    this->declare_parameter("resample_max_weight_threshold", RESAMPLE_MAX_WEIGHT_THRESHOLD);
    this->declare_parameter("inject_num_iterations", INJECT_NUM_ITERATIONS);
    this->declare_parameter("inject_percentage", INJECT_PERCENTAGE);
    this->declare_parameter("replace_worst_percentage", REPLACE_WORST_PERCENTAGE);
    this->declare_parameter("estimate_num_particles", ESTIMATE_NUM_PARTICLES);
    this->declare_parameter("map_features", std::string(""));
    this->declare_parameter("map_yaml", std::string(""));
    this->declare_parameter("map_pgm", std::string(""));

    success &= this->get_parameter("num_particles", num_particles_);
    success &= this->get_parameter("room_size_x", room_size_x_);
    success &= this->get_parameter("room_size_y", room_size_y_);
    success &= this->get_parameter("motion_delta_distance", motion_delta_distance_);
    success &= this->get_parameter("motion_delta_angle", motion_delta_angle_);
    success &= this->get_parameter("motion_x_variance", motion_x_variance_);
    success &= this->get_parameter("motion_y_variance", motion_y_variance_);
    success &= this->get_parameter("motion_angle_variance", motion_angle_variance_);
    success &= this->get_parameter("resample_ess_threshold", resample_ess_threshold_);
    success &= this->get_parameter("resample_max_weight_threshold", resample_max_weight_threshold_);
    success &= this->get_parameter("inject_num_iterations", inject_num_iterations_);
    success &= this->get_parameter("inject_percentage", inject_percentage_);
    success &= this->get_parameter("replace_worst_percentage", replace_worst_percentage_);
    success &= this->get_parameter("estimate_num_particles", estimate_num_particles_);
    success &= this->get_parameter("map_features", map_features_);
    success &= this->get_parameter("map_yaml", map_yaml_);
    success &= this->get_parameter("map_pgm", map_pgm_);

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "One or more parameters failed to load. Check your YAML or launch file.");
        // Optionally shutdown or throw an exception here
        return;
    }

    // Log loaded parameters
    RCLCPP_INFO(this->get_logger(), "num_particles: %.1f", num_particles_);
    RCLCPP_INFO(this->get_logger(), "motion_delta_distance: %.2f", motion_delta_distance_);
    RCLCPP_INFO(this->get_logger(), "motion_delta_angle: %.2f", motion_delta_angle_);
    RCLCPP_INFO(this->get_logger(), "motion_x_variance: %.2f", motion_x_variance_);
    RCLCPP_INFO(this->get_logger(), "motion_y_variance: %.2f", motion_y_variance_);
    RCLCPP_INFO(this->get_logger(), "motion_angle_variance: %.2f", motion_angle_variance_);
    RCLCPP_INFO(this->get_logger(), "resample_ess_threshold: %.2f", resample_ess_threshold_);
    RCLCPP_INFO(this->get_logger(), "resample_max_weight_threshold: %.2f", resample_max_weight_threshold_);
    RCLCPP_INFO(this->get_logger(), "inject_num_iterations: %d", inject_num_iterations_);
    RCLCPP_INFO(this->get_logger(), "inject_percentage: %.2f", inject_percentage_);
    RCLCPP_INFO(this->get_logger(), "replace_worst_percentage: %.2f", replace_worst_percentage_);
    RCLCPP_INFO(this->get_logger(), "estimate_num_particles: %d", estimate_num_particles_);
    RCLCPP_INFO(this->get_logger(), "map_features: %s", map_features_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_yaml: %s", map_yaml_.c_str());
    RCLCPP_INFO(this->get_logger(), "map_pgm: %s", map_pgm_.c_str());
}

void ParticleFilter::calculateFreeSpaceFromPGM()
{
    // Load map.yaml
    YAML::Node config = YAML::LoadFile(map_yaml_);
    resolution = config["resolution"].as<double>();
    origin = config["origin"].as<std::vector<double>>();
    int negate = config["negate"] ? config["negate"].as<int>() : 0;

    // Load PGM
    // PGMImage pgm;
    pgm.load(map_pgm_);

    // Collect free pixels
    // std::vector<std::pair<int, int>> free_pixels;
    for (int y = 0; y < pgm.height; ++y)
    {
        for (int x = 0; x < pgm.width; ++x)
        {
            uint8_t val = pgm.pixel(x, y);
            bool is_free = (negate == 0) ? (val >= 254) : (val <= 1);
            if (is_free)
            {
                free_pixels.emplace_back(x, y);
            }
        }
    }
}

// normalize the weights of the particles
void ParticleFilter::normalizeWeights()
{
    double sum_weights = std::accumulate(particles_.begin(), particles_.end(), 0.0,
                                         [](double sum, const Particle &p)
                                         { return sum + p.weight; });

    for (auto &p : particles_)
    {
        p.weight /= sum_weights;
    }
}

// get the maximum weight of the particles
double ParticleFilter::maxWeight()
{
    double max_weight = 0.0;
    for (const auto &p : particles_)
    {
        max_weight = std::max(max_weight, p.weight);
    }
    return max_weight;
}

// compute the color weight lookup table for visualization
void ParticleFilter::computeColorWeightLookup()
{
    double average_weight = 1.0 / num_particles_;

    ColorWeightLookup = {
        {0.9 * average_weight, {1.0, 1.0, 1.0}},              // White
        {average_weight, {0.56, 0.0, 1.0}},                   // Violet
        {1.05 * average_weight, {0.0, 0.0, 1.0}},             // Blue
        {1.1 * average_weight, {0.0, 1.0, 0.5}},              // Cyan
        {1.15 * average_weight, {0.0, 1.0, 0.0}},             // Green
        {1.2 * average_weight, {1.0, 1.0, 0.0}},              // Yellow
        {1.5 * average_weight, {1.0, 0.5, 0.0}},              // Orange
        {std::numeric_limits<double>::max(), {1.0, 0.0, 0.0}} // Red
    };
}

// get color based on the weight of the particle
std::vector<double> ParticleFilter::colorFromWeight(double weight) const
{
    for (const auto &entry : ColorWeightLookup)
    {
        if (weight < entry.first)
        {
            return entry.second;
        }
    }
    RCLCPP_WARN(this->get_logger(), "Weight out of range: %f", weight);
    return {1.0, 0.0, 0.0}; // Default to Red
}

// publish the particles for visualization as markers
void ParticleFilter::publishParticles()
{
    if (particles_.empty())
        return;

    visualization_msgs::msg::MarkerArray marker_array;
    int i = 0;

    for (const auto &p : particles_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "particle";
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

        auto color = colorFromWeight(p.weight);
        marker.color.a = 1.0;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];

        marker_array.markers.push_back(marker);
    }

    particles_pub_->publish(marker_array);
}

// replace the worst particles with random ones in white part of pgm (free space)
void ParticleFilter::replaceWorstParticles_pgm(double percentage)
{
    std::sort(particles_.begin(), particles_.end(),
              [](const Particle &a, const Particle &b)
              { return a.weight < b.weight; });

    int num_replace = static_cast<int>(num_particles_ * percentage);

    // Sample random particles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> index_dist(0, free_pixels.size() - 1);

    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    double init_weight = 1.0 / num_particles_;

    for (int i = 0; i < num_replace; i++)
    {
        auto [x_pix, y_pix] = free_pixels[index_dist(gen)];
        auto [x, y] = pixelToWorld(x_pix, y_pix, resolution, origin, pgm.height);

        particles_[i].x = x;
        particles_[i].y = y;
        particles_[i].theta = dist_theta(generator_);
        particles_[i].weight = init_weight;
    }

    normalizeWeights();
}

// replace and inject random particles into the filter in white part of pgm (free space)
void ParticleFilter::injectRandomParticles_pgm(double percentage)
{
    // replace random particles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> index_dist(0, free_pixels.size() - 1);

    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

    int num_replace = static_cast<int>(num_particles_ * percentage);

    std::vector<int> random_indices(num_replace);
    for (int &index : random_indices)
    {
        index = rand() % static_cast<int>(num_particles_);
    }

    double init_weight = 1.0 / num_particles_;

    for (int i : random_indices)
    {
        auto [x_pix, y_pix] = free_pixels[index_dist(gen)];
        auto [x, y] = pixelToWorld(x_pix, y_pix, resolution, origin, pgm.height);

        particles_[i].x = x;
        particles_[i].y = y;
        particles_[i].theta = dist_theta(generator_);
        particles_[i].weight = init_weight;
    }
}

#pragma endregion auxiliar functions

//! auxiliar functions end!//

//! Feature Handling start !//

#pragma region feature handling

// store the map message received from the topic
void ParticleFilter::storeMapMessage(const robot_msgs::msg::FeatureArray::SharedPtr msg)
{
    last_map_msg_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received features");
    new_map = true;
}

std::vector<map_features::Feature> ParticleFilter::getExpectedFeatures(const Particle &p, const std::string &type)
{
    std::vector<map_features::Feature> features_particle;

    double cos_theta = std::cos(p.theta);
    double sin_theta = std::sin(p.theta);

    for (const auto &feature_ptr : global_features_)
    {
        if (feature_ptr->type == type)
        {
            auto object_ptr = std::dynamic_pointer_cast<map_features::Feature>(feature_ptr);
            if (!object_ptr)
                continue;

            double map_x = object_ptr->x;
            double map_y = object_ptr->y;
            double feature_theta = object_ptr->theta;

            double particle_x = cos_theta * (map_x - p.x) + sin_theta * (map_y - p.y);
            double particle_y = -sin_theta * (map_x - p.x) + cos_theta * (map_y - p.y);

            features_particle.emplace_back(particle_x, particle_y, feature_theta, type);
        }
    }

    return features_particle;
}

// transform angle from the map frame to the particle frame
double ParticleFilter::transformAngleToParticleFrame(double feature_theta_map, double particle_theta)
{

    // print the theta of the feature and the particle
    RCLCPP_INFO(this->get_logger(), "Feature theta: %.2f, Particle theta: %.2f", feature_theta_map, particle_theta);

    if (particle_theta < -M_PI)
        particle_theta += 2 * M_PI;
    else if (particle_theta > M_PI)
        particle_theta -= 2 * M_PI;

    feature_theta_map = feature_theta_map * (M_PI / 180.0);
    if (feature_theta_map < -M_PI)
        feature_theta_map += 2 * M_PI;
    else if (feature_theta_map > M_PI)
        feature_theta_map -= 2 * M_PI;

    // compute the angle between the feature and the particle

    double angle = feature_theta_map - particle_theta;

    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;

    return angle;
}

// compute the likelihood for the orientation of the corner feature
double ParticleFilter::computeAngleLikelihood(double measured_angle, double expected_angle, double sigma)
{
    if (measured_angle > M_PI)
        measured_angle -= 2 * M_PI;
    if (measured_angle < -M_PI)
        measured_angle += 2 * M_PI;
    if (expected_angle > M_PI)
        expected_angle -= 2 * M_PI;
    if (expected_angle < -M_PI)
        expected_angle += 2 * M_PI;

    double error = measured_angle - expected_angle;

    while (error > M_PI)
        error -= 2 * M_PI;
    while (error < -M_PI)
        error += 2 * M_PI;

    double coeff = 1.0 / std::sqrt(2.0 * M_PI * sigma * sigma);
    double exponent = -0.5 * (error * error) / (sigma * sigma);

    return coeff * std::exp(exponent);
}

// compute the likelihood of a corner feature based on distance and angle
double ParticleFilter::computeLikelihoodFeature(const Particle &p, double noisy_x, double noisy_y, double measured_theta, double sigma_pos, double sigma_theta, const std::string &type)
{
    std::vector<map_features::Feature> expected_features = getExpectedFeatures(p, type);

    double min_dist = std::numeric_limits<double>::max();
    map_features::Feature best_feature(0, 0, 0, type);

    double likelihood = 0.0;

    for (const auto &exp : expected_features)
    {
        double dist = std::hypot(noisy_x - exp.x, noisy_y - exp.y);
        if (dist < min_dist)
        {
            min_dist = dist;
            best_feature = exp;
        }
    }

    // Compute likelihood based on distance and angle
    double expected_feature_angle = transformAngleToParticleFrame(best_feature.theta, p.theta);
    double angle_likelihood = computeAngleLikelihood(measured_theta, expected_feature_angle, sigma_theta);
    double distance_likelihood = (std::exp(-(min_dist * min_dist) / (2 * sigma_pos * sigma_pos))) / std::sqrt(2 * M_PI * sigma_pos * sigma_pos);

    if (with_angle_)
    {
        likelihood = (angle_likelihood + distance_likelihood);
    }
    else
    {
        likelihood = distance_likelihood;
    }

    return likelihood;
}

// decode a feature message received from the topic features into a DecodedMsg structure
ParticleFilter::DecodedMsg ParticleFilter::decodeMsg(const robot_msgs::msg::Feature &msg)
{
    DecodedMsg feature;

    feature.x = msg.x;
    feature.y = msg.y;
    feature.theta = msg.theta;
    feature.type = msg.type;
    feature.confidence = msg.confidence;
    feature.angle_variance = msg.orientation_variance;

    for (size_t i = 0; i < 2; ++i)
    {
        for (size_t j = 0; j < 2; ++j)
        {
            feature.covariance_pos[i][j] = msg.position_covariance[i * 3 + j];
        }
    }


    return feature;
}

#pragma endregion feature handling

//! Feature Handling end !//

//! Resampling functions start !//

#pragma region resampling functions

void ParticleFilter::multinomialResample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < num_particles_; i++)
    {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles_[i].weight;
    }

    std::uniform_real_distribution<double> dist(0.0, cumulative_weights.back());
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);

    for (size_t i = 0; i < num_particles_; i++)
    {
        double r = dist(generator_);
        auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), r);
        int index = std::distance(cumulative_weights.begin(), it);
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;
}

void ParticleFilter::stratifiedResample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < num_particles_; i++)
    {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles_[i].weight;
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);

    int index = 0;
    for (size_t i = 0; i < num_particles_; i++)
    {
        double r = dist(generator_); // Generate new random variable for each particle
        double U = r + (i / static_cast<double>(num_particles_));
        while (U > cumulative_weights[index])
            index++;
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;
}

void ParticleFilter::systematicResample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    // Compute cumulative weights
    std::vector<double> cumulative_weights(num_particles_);
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < num_particles_; i++)
    {
        cumulative_weights[i] = cumulative_weights[i - 1] + particles_[i].weight;
    }

    std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
    double r = dist(generator_);
    int index = 0;
    for (size_t i = 0; i < num_particles_; i++)
    {
        double U = r + (i / static_cast<double>(num_particles_));
        while (U > cumulative_weights[index])
            index++;
        new_particles.push_back(particles_[index]);
    }

    particles_ = new_particles;
}

void ParticleFilter::residualResample()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    std::vector<double> residual_weights;
    int total_copies = 0;

    for (const auto &p : particles_)
    {
        int num_copies = static_cast<int>(p.weight * num_particles_);
        total_copies += num_copies;
        for (int j = 0; j < num_copies; j++)
            new_particles.push_back(p);
        residual_weights.push_back((p.weight * num_particles_) - num_copies);
    }

    std::vector<double> cumulative_weights;
    double sum_residuals = 0.0;
    for (double rw : residual_weights)
    {
        sum_residuals += rw;
        cumulative_weights.push_back(sum_residuals);
    }

    std::uniform_real_distribution<double> dist(0.0, sum_residuals);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_.seed(seed);
    while (total_copies < num_particles_)
    {
        double r = dist(generator_);
        for (size_t i = 0; i < cumulative_weights.size(); i++)
        {
            if (r <= cumulative_weights[i])
            {
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

void ParticleFilter::initializeParticles_pgm()
{
    // Sample random particles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> index_dist(0, free_pixels.size() - 1);
    std::uniform_real_distribution<> theta_dist(-M_PI, M_PI);
    std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);
    double init_weight = 1.0 / num_particles_;

    particles_.resize(num_particles_);
    for (auto &p : particles_)
    {
        auto [x_pix, y_pix] = free_pixels[index_dist(gen)];
        auto [x, y] = pixelToWorld(x_pix, y_pix, resolution, origin, pgm.height);

        geometry_msgs::msg::Pose pose;
        p.x = x;
        p.y = y;
        p.theta = dist_theta(generator_);
        p.weight = init_weight;
    }

    /* while(1){
        publishParticles();
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } */

    /* if (free_pixels.size() < static_cast<size_t>(num_particles_)) {
        throw std::runtime_error("Not enough free pixels");
    } */

    /* while(1){
        publishParticles();
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } */

    /*  for (auto &p : particles_) {
         p.x = dist_x(generator_);
         p.y = dist_y(generator_);
         p.theta = dist_theta(generator_);
         p.weight = 1.0 / num_particles_;
     }  */
}

void ParticleFilter::motionUpdate(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (particles_.empty())
    {
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
        msg->pose.pose.orientation.w);

    double roll, pitch, odom_theta;
    tf2::Matrix3x3(odom_q).getRPY(roll, pitch, odom_theta);

    double delta_x_odom = odom_x - last_x_;
    double delta_y_odom = odom_y - last_y_;
    double delta_distance = std::hypot(delta_x_odom, delta_y_odom);

    std::uniform_real_distribution<double> noise_x(-motion_x_variance_, motion_x_variance_);
    std::uniform_real_distribution<double> noise_y(-motion_y_variance_, motion_y_variance_);
    std::uniform_real_distribution<double> noise_theta(-motion_angle_variance_, motion_angle_variance_);

    double alpha_odom = atan2(delta_y_odom, delta_x_odom);
    double alpha_robot = alpha_odom - last_theta_;
    double delta_x_robot = delta_distance * std::cos(alpha_robot);
    double delta_y_robot = delta_distance * std::sin(alpha_robot);
    double delta_theta_odom = odom_theta - last_theta_;

    // update particles if significant motion is detected
    if (delta_distance > motion_delta_distance_ || std::abs(delta_theta_odom) > motion_delta_angle_)
    {
        if (!last_map_msg_)
        {
            RCLCPP_WARN(this->get_logger(), "No keypoint message available yet.");
            return;
        }

        for (auto &p : particles_)
        {
            p.x += delta_x_robot * std::cos(p.theta) - delta_y_robot * std::sin(p.theta) + noise_x(generator_);
            p.y += delta_x_robot * std::sin(p.theta) + delta_y_robot * std::cos(p.theta) + noise_y(generator_);
            p.theta += delta_theta_odom + noise_theta(generator_);

            if (p.theta > M_PI)
                p.theta -= 2 * M_PI;
            if (p.theta < -M_PI)
                p.theta += 2 * M_PI;
        }

        last_x_ = odom_x;
        last_y_ = odom_y;
        last_theta_ = odom_theta;

        // update the particles weights
        measurementUpdate(last_map_msg_);
    }

    publishParticles();
}

void ParticleFilter::measurementUpdate(const robot_msgs::msg::FeatureArray::SharedPtr msg)
{
    if (particles_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No particles to update.");
        return;
    }

    if (!new_map)
    {
        return;
    }
    new_map = false;

    bool all_outside = true;

    for (auto &p : particles_)
    {
        double likelihood = 0;

        for (const auto &obs_msg : msg->features)
        {
            DecodedMsg obs = decodeMsg(obs_msg);

            double sigma_x = std::sqrt(obs.covariance_pos[0][0]);
            double sigma_y = std::sqrt(obs.covariance_pos[1][1]);
            double sigma_theta = std::sqrt(obs.angle_variance);
            double sigma_pos = std::sqrt((sigma_x * sigma_x + sigma_y * sigma_y) / 2.0);

            //! VER ISTO (os noises não são só para o calculo da likelihood?)
            /*std::normal_distribution<double> noise_pos_x(0.0, sigma_x);
            std::normal_distribution<double> noise_pos_y(0.0, sigma_y);
            std::normal_distribution<double> noise_pos_z(0.0, sigma_z);
            std::normal_distribution<double> noise_theta(0.0, sigma_theta);

            double noisy_x = obs.x + noise_pos_x(generator_);
            double noisy_y = obs.y + noise_pos_y(generator_);
            double noisy_z = obs.z + noise_pos_z(generator_);
            double measured_theta = obs.theta + noise_theta(generator_); */
            // ! ########

            // Compute likelihood based on feature type

            likelihood += computeLikelihoodFeature(p, obs.x, obs.y, obs.theta, sigma_pos, sigma_theta, obs.type);
        }

        p.weight *= likelihood;

        bool penalize = !isParticleInFreeSpace(p.x, p.y, pgm, resolution, origin);
        if (penalize)
        {
            p.weight = p.weight / 2;
        }
        else
        {
            all_outside = false;
        }
    }

    if (all_outside == true)
    {
        RCLCPP_WARN(this->get_logger(), "All particles are outside the free space.");
        injectRandomParticles_pgm(1);
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Not all particles are outside the free space.");
        normalizeWeights();
    }

    // perform resampling
    resampleParticles(ResamplingAmount::ESS, ResamplingMethod::RESIDUAL);

    // Replace worst particles if resampling flag is not set
    if (!resample_flag_)
    {
        // replaceWorstParticles(replace_worst_percentage_);
        replaceWorstParticles_pgm(replace_worst_percentage_);
    }
    else
    {
        resample_flag_ = false;
    }
}

void ParticleFilter::resampleParticles(ResamplingAmount type, ResamplingMethod method)
{
    if (particles_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No particles to resample.");
        return;
    }

    double max_weight = maxWeight();
    double ess = 1.0 / std::accumulate(particles_.begin(), particles_.end(), 0.0,
                                       [](double sum, const Particle &p)
                                       { return sum + (p.weight * p.weight); });

    RCLCPP_INFO(this->get_logger(), "Max weight: %f, ESS: %f", max_weight, ess);

    switch (type)
    {
    case ResamplingAmount::ESS:
        if (ess > num_particles_ * resample_ess_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "Skipping resampling, particles are well-distributed.");
            return;
        }
        break;
    case ResamplingAmount::MAX_WEIGHT:
        if (max_weight < resample_max_weight_threshold_ / num_particles_)
        {
            RCLCPP_INFO(this->get_logger(), "Skipping resampling, max weight is high.");
            return;
        }
        break;
    }

    resample_flag_ = true;

    // Perform resampling based on the specified method
    switch (method)
    {
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

    // Reset particle weights after resampling
    for (auto &p : particles_)
    {
        p.weight = 1.0 / num_particles_;
    }

    // Inject random particles base on number of resamples performed
    iterationCounter++;
    if (iterationCounter == inject_num_iterations_)
    {
        RCLCPP_INFO(this->get_logger(), "Injecting random particles.");
        // injectRandomParticles(inject_percentage_);
        injectRandomParticles_pgm(inject_percentage_);

        iterationCounter = 0;
    }
}

// compute the estimated pose based on the top-weighted particles
void ParticleFilter::computeEstimatedPose()
{
    if (particles_.empty())
        return;

    std::vector<Particle> sorted_particles = particles_;
    std::sort(sorted_particles.begin(), sorted_particles.end(),
              [](const Particle &a, const Particle &b)
              {
                  return a.weight > b.weight;
              });

    // Use only the top estimate_num_particles_ particles
    int num_top_particles = std::min(estimate_num_particles_, static_cast<int>(sorted_particles.size()));

    double x_sum = 0, y_sum = 0, theta_sum = 0, theta_x_sum = 0, theta_y_sum = 0, weight_sum = 0;

    for (int i = 0; i < num_top_particles; i++)
    {
        const auto &p = sorted_particles[i];
        x_sum += p.x * p.weight;
        y_sum += p.y * p.weight;
        theta_sum += p.theta * p.weight;
        theta_x_sum += std::cos(p.theta) * p.weight;
        theta_y_sum += std::sin(p.theta) * p.weight;
        weight_sum += p.weight;
    }

    if (weight_sum > 0)
    {
        x_sum /= weight_sum;
        y_sum /= weight_sum;
        theta_sum /= weight_sum;
        theta_x_sum /= weight_sum;
        theta_y_sum /= weight_sum;
    }

    // Compute the final estimated pose
    x_last_final = x_sum;
    y_last_final = y_sum;
    theta_last_final = std::atan2(theta_y_sum, theta_x_sum);

    if (theta_last_final > M_PI)
        theta_last_final -= 2 * M_PI;
    if (theta_last_final < -M_PI)
        theta_last_final += 2 * M_PI;
}

// publish the estimated pose and the map to odom transform
void ParticleFilter::publishEstimatedPose()
{
    if (particles_.empty())
        return;

    computeEstimatedPose();

    if (!msg_odom_base_link_)
    {
        RCLCPP_WARN(this->get_logger(), "Skipping pose publication: No odometry data available.");
        return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = x_last_final;
    pose_msg.pose.pose.position.y = y_last_final;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_last_final);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);

    // Publish `map -> base_link` transform
    geometry_msgs::msg::TransformStamped map_to_pose_tf;
    map_to_pose_tf.header.stamp = this->get_clock()->now();
    map_to_pose_tf.header.frame_id = "map";
    map_to_pose_tf.child_frame_id = "estimated_pose";

    // Calculate map -> odom transform
    map_to_pose_tf.transform.translation.x = x_last_final;
    map_to_pose_tf.transform.translation.y = y_last_final;
    map_to_pose_tf.transform.translation.z = 0.0;

    map_to_pose_tf.transform.rotation.x = q.x();
    map_to_pose_tf.transform.rotation.y = q.y();
    map_to_pose_tf.transform.rotation.z = q.z();
    map_to_pose_tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(map_to_pose_tf);

    // RCLCPP_INFO(this->get_logger(), "Published estimated pose (Top 10 weighted particles).");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParticleFilter>());
    rclcpp::shutdown();
    return 0;
}

#pragma endregion pf functions

//! Particle Filter Functions !//