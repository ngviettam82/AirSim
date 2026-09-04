#include "px4_airsim_autonomy/algorithm_factory.hpp"
#include "px4_airsim_autonomy/enterprise/photogrammetry_survey_mission.hpp"
#include "px4_airsim_autonomy/enterprise/dynamic_avoidance_mission.hpp"

namespace px4_airsim_autonomy {

std::shared_ptr<IAutonomyAlgorithm> AlgorithmFactory::createAlgorithm(const std::string& name)
{
    if (name.empty() || name == "none" || name == "blank") {
        return nullptr;
    }

    if (name == "photogrammetry_survey" || name == "survey" || name == "scanning_patrol" || name == "scanning") {
        return std::make_shared<enterprise::PhotogrammetrySurveyMission>();
    } else if (name == "dynamic_avoidance" || name == "avoidance" || name == "obstacle_avoidance") {
        return std::make_shared<enterprise::DynamicAvoidanceMission>();
    }

    return nullptr;
}

std::vector<std::string> AlgorithmFactory::getAvailableAlgorithms()
{
    return {
        "photogrammetry_survey",
        "dynamic_avoidance"
    };
}

} // namespace px4_airsim_autonomy
