#pragma once
#include "robot/types.hpp"
#include <vector>

namespace robot
{

    // Computes curvature at index i using points i-1, i, i+1.
    // Returns 0.0 if not computable.
    double curvatureAt(const std::vector<Vec2> &path, size_t i);

    // Computes curvature for the whole path.
    std::vector<double> curvatureProfile(const std::vector<Vec2> &path);

}