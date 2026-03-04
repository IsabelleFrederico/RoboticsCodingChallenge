#pragma once
#include "robot/types.hpp"
#include "robot/speed_profile.hpp"

namespace robot
{

    // Computes total travel time [s] for the given path,
    // using curvature -> speed and dt = ds / v_segment.
    double totalTimeSeconds(const std::vector<Vec2> &path, const SpeedParams &params);

} // namespace robot