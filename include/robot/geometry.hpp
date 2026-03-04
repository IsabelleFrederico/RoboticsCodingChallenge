#pragma once
#include "robot/types.hpp"

namespace robot
{

    double distance(const Vec2 &a, const Vec2 &b);

    double pathLength(const std::vector<Vec2> &path);

    double gadgetWidth(const InputData &data);

}