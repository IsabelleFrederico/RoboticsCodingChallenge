#pragma once
#include "robot/types.hpp"
#include <vector>

namespace robot
{

    double curvatureAt(const std::vector<Vec2> &path, size_t i);

    std::vector<double> curvatureProfile(const std::vector<Vec2> &path);

}