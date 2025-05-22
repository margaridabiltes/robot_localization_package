#include "robot_localization_package/MapLoader.hpp"

namespace map_features
{

    // Static member definition
    std::vector<FeaturePtr> MapLoader::global_features_;

    void MapLoader::loadToGlobalMap(const std::string &yaml_path)
    {
        // Check if the file exists
        std::ifstream file(yaml_path);
        if (!file.good())
        {
            throw std::runtime_error("YAML file not found: " + yaml_path);
        }

        // Load the YAML file
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root["features"])
        {
            throw std::runtime_error("No 'features' key found in YAML file: " + yaml_path);
        }

        for (const auto &f : root["features"])
        {
            std::string type = f["type"].as<std::string>();
            double x = f["position"]["x"].as<double>();
            double y = f["position"]["y"].as<double>();
            double theta = f["orientation"]["theta"].as<double>();

            auto feature = std::make_shared<Feature>(x, y, theta, type);
            addToGlobalMap(feature);
        }
    }

    void MapLoader::addToGlobalMap(FeaturePtr feature)
    {
        global_features_.emplace_back(feature);
    }

    const std::vector<FeaturePtr> &MapLoader::getGlobalFeatureMap()
    {
        return global_features_;
    }

} // namespace map_features
