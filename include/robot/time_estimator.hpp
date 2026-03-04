#pragma once
#include "robot/types.hpp"
#include "robot/speed_profile.hpp"

namespace robot
{

    double totalTimeSeconds(const std::vector<Vec2> &path, const SpeedParams &params);

}