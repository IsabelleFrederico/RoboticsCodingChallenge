#pragma once
#include "robot/types.hpp"
#include <vector>

namespace robot
{
    double uniqueCleanedAreaGrid(const std::vector<Vec2> &path,
                                 double gadgetWidth,
                                 double cellSize);
}