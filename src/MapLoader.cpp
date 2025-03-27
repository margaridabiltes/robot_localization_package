#include "robot_localization_package/MapLoader.hpp"

namespace map_features {

// Static member definition
std::vector<FeaturePtr> MapLoader::global_features_;

void MapLoader::loadToGlobalMap(const std::string& yaml_path) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["features"]) {
        throw std::runtime_error("No 'features' key found in YAML.");
    }

    for (const auto& f : root["features"]) {
        std::string type = f["type"].as<std::string>();
        double x = f["position"]["x"].as<double>();
        double y = f["position"]["y"].as<double>();
        double z = f["position"]["z"].as<double>();
        double theta = f["orientation"]["theta"].as<double>();

        if (type == "corner") {
            auto feature = std::make_shared<FeatureCorner>(x, y,z, theta);
            addToGlobalMap(feature);
        } else {
            //get keypoints
            std::vector<geometry_msgs::msg::Point> keypoints;
            for (const auto& kp : f["keypoints"]) {
                geometry_msgs::msg::Point point;
                point.x = kp["x"].as<double>();
                point.y = kp["y"].as<double>();
                point.z = kp["z"].as<double>();
                keypoints.push_back(point);
            }
            auto feature = std::make_shared<FeatureObject>(x, y, z, theta, type, keypoints);
            addToGlobalMap(feature);
        }
    }
}

void MapLoader::addToGlobalMap(FeaturePtr feature) {
    global_features_.emplace_back(feature);
}

const std::vector<FeaturePtr>& MapLoader::getGlobalFeatureMap() {
    return global_features_;
}

}  // namespace map_features
