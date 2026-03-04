#include <iostream>
#include <cmath>
#include "robot/curvature.hpp"

static bool approx(double a, double b, double eps = 1e-6)
{
    return std::abs(a - b) < eps;
}

int main()
{
    // Straight line → curvature should be 0
    std::vector<robot::Vec2> path{
        {0, 0},
        {1, 0},
        {2, 0}};

    double k = robot::curvatureAt(path, 1);

    if (!approx(k, 0.0))
    {
        std::cerr << "Curvature test failed. Got " << k << "\n";
        return 1;
    }

    std::cout << "Curvature test passed.\n";
    return 0;
}