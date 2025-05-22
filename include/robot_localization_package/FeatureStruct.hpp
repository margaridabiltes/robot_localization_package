#pragma once

#include <string>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace map_features
{

    // Base class for all map features
    struct Feature
    {
        double x;         // X-coordinate of the feature
        double y;         // Y-coordinate of the feature
        double theta;     // Orientation of the feature
        std::string type; // Type of the feature (e.g., "corner", "object")

        // Constructor
        Feature(double x_, double y_, double theta_, const std::string &type_)
            : x(x_), y(y_), theta(theta_), type(type_) {}

        // Virtual destructor for proper cleanup of derived classes
        virtual ~Feature() = default;
    };

    using FeaturePtr = std::shared_ptr<Feature>;

} // namespace map_features