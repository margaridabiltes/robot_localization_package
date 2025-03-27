#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <yaml-cpp/yaml.h>

#include "FeatureStruct.hpp"

namespace map_features {

class MapLoader {
public:
    void loadToGlobalMap(const std::string& yaml_path);

    static const std::vector<FeaturePtr>& getGlobalFeatureMap();

private:
    static void addToGlobalMap(FeaturePtr feature);

    static std::vector<FeaturePtr> global_features_;
};

}  // namespace map_features
