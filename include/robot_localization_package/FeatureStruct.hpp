#pragma once

#include <string>
#include <memory>

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


struct FeatureSquare : public Feature {
    double side_length;
    double rotation_deg;  

    FeatureSquare(double x_, double y_, double z_, double theta_, double side_length_, double rotation_deg_)
        : Feature(x_, y_,z_, theta_, "square"),
          side_length(side_length_),
          rotation_deg(rotation_deg_) {}
};

using FeaturePtr = std::shared_ptr<Feature>;

}  // namespace map_features
