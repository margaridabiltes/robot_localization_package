#pragma once

#include <string>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/point.hpp> 
#include <geometry_msgs/msg/quaternion.hpp>

namespace map_features {

// Base class for all map features
struct Feature {
    geometry_msgs::msg::Point position;             // Position of the feature (x, y, z)
    geometry_msgs::msg::Quaternion orientation;     // Orientation of the feature (quaternion)
    std::string type; // Type of the feature ("corner", "square", etc.)

    // Constructor
    Feature(const geometry_msgs::msg::Point& position_, const geometry_msgs::msg::Quaternion& orientation_, const std::string& type_)
        : position(position_), orientation(orientation_), type(type_) {}

    virtual ~Feature() = default;
};

// Derived class for corner features
struct FeatureCorner : public Feature {
    // Constructor
    FeatureCorner(const geometry_msgs::msg::Point& position_, const geometry_msgs::msg::Quaternion& orientation_)
        : Feature(position_, orientation_, "corner") {}
};

// Derived class for object features
struct FeatureObject : public Feature {
    std::vector<geometry_msgs::msg::Point> keypoints; // Keypoints associated with the center of mass of the object

    // Constructor
    FeatureObject(const geometry_msgs::msg::Point& position_, const geometry_msgs::msg::Quaternion& orientation_, const std::string& type_, const std::vector<geometry_msgs::msg::Point>& keypoints_)
        : Feature(position_, orientation_, type_), keypoints(keypoints_) {}
};


using FeaturePtr = std::shared_ptr<Feature>;

}  // namespace map_features