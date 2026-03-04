#pragma once
#include "robot/types.hpp"
#include <vector>

namespace robot
{
    // Approximates unique cleaned area by rasterizing into a grid of cellSize x cellSize.
    // A cell is counted as cleaned if the cleaning disk (radius = gadgetWidth/2) intersects it
    // anywhere along the path.
    double uniqueCleanedAreaGrid(const std::vector<Vec2> &path,
                                 double gadgetWidth,
                                 double cellSize);
}