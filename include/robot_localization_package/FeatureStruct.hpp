#pragma once

#include <string>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/point.hpp> 

namespace map_features {

// Base class for all map features
struct Feature {
    double x;       // X-coordinate of the feature
    double y;       // Y-coordinate of the feature
    double z;       // Z-coordinate of the feature
    double theta;   // Orientation of the feature
    std::string type; // Type of the feature (e.g., "corner", "object")

    // Constructor
    Feature(double x_, double y_, double z_, double theta_, const std::string& type_)
        : x(x_), y(y_), z(z_), theta(theta_), type(type_) {}

    // Virtual destructor for proper cleanup of derived classes
    virtual ~Feature() = default;
};

// Derived class for corner features
struct FeatureCorner : public Feature {
    // Constructor
    FeatureCorner(double x_, double y_, double z_, double theta_)
        : Feature(x_, y_, z_, theta_, "corner") {}
};

// Derived class for object features
struct FeatureObject : public Feature {
    std::vector<geometry_msgs::msg::Point> keypoints; // Keypoints associated with the object

    // Constructor
    FeatureObject(double x_, double y_, double z_, double theta_, const std::string& type_, const std::vector<geometry_msgs::msg::Point>& keypoints_)
        : Feature(x_, y_, z_, theta_, type_), keypoints(keypoints_) {}
};

using FeaturePtr = std::shared_ptr<Feature>;

}  // namespace map_features