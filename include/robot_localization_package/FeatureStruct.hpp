#pragma once

#include <string>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/point.hpp> 


namespace map_features {

struct Feature {
    double x;
    double y;
    double z;
    double theta;  
    std::string type;

    Feature(double x_, double y_, double z_, double theta_, const std::string& type_)
        : x(x_), y(y_),z(z_), theta(theta_), type(type_) {}

    virtual ~Feature() = default;
};

struct FeatureCorner : public Feature {
    FeatureCorner(double x_, double y_,double z_, double theta_)
        : Feature(x_, y_,z_, theta_, "corner") {}
};


struct FeatureObject : public Feature {
    std::vector<geometry_msgs::msg::Point> keypoints; 

    FeatureObject(double x_, double y_, double z_, double theta_, const std::string& type_, const std::vector<geometry_msgs::msg::Point>& keypoints_)
        : Feature(x_, y_, z_, theta_, type_), keypoints(keypoints_) {}
};

using FeaturePtr = std::shared_ptr<Feature>;

}  // namespace map_features
