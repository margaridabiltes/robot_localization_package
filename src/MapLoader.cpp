#include "robot_localization_package/MapLoader.hpp"

namespace map_features {

// Static member definition
std::vector<FeaturePtr> MapLoader::global_features_;

void MapLoader::loadToGlobalMap(const std::string& yaml_path) {
        // Check if the file exists
        std::ifstream file(yaml_path);
        if (!file.good()) {
            throw std::runtime_error("YAML file not found: " + yaml_path);
        }
    
        // Load the YAML file
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root["features"]) {
            throw std::runtime_error("No 'features' key found in YAML file: " + yaml_path);
        }

    for (const auto& f : root["features"]) {
        std::string type = f["type"].as<std::string>();

        // Extract position
        geometry_msgs::msg::Point position;
        position.x = f["position"]["x"].as<double>();
        position.y = f["position"]["y"].as<double>();
        position.z = f["position"]["z"].as<double>();

        // Extract orientation (quaternion)
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = f["orientation"]["x"].as<double>();
        orientation.y = f["orientation"]["y"].as<double>();
        orientation.z = f["orientation"]["z"].as<double>();
        orientation.w = f["orientation"]["w"].as<double>();
        
        if (type == "corner") {
            auto feature = std::make_shared<FeatureCorner>(position, orientation);
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
            auto feature = std::make_shared<FeatureObject>(position, orientation, type, keypoints);
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
