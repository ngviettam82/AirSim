#pragma once

#include "algorithm_base.hpp"
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace px4_airsim_autonomy {

class AlgorithmFactory {
public:
    static std::shared_ptr<IAutonomyAlgorithm> createAlgorithm(const std::string& name);
    static std::vector<std::string> getAvailableAlgorithms();
};

} // namespace px4_airsim_autonomy

