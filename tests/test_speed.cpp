#include <iostream>
#include <cmath>
#include "robot/speed_profile.hpp"

static bool approx(double a, double b, double eps = 1e-6)
{
    return std::abs(a - b) < eps;
}

int main()
{
    robot::SpeedParams p;

    if (!approx(robot::speedFromCurvature(0.0, p), p.vmax))
        return 1;

    if (!approx(robot::speedFromCurvature(p.kmax, p), p.vmin))
        return 1;

    std::cout << "Speed test passed.\n";
    return 0;
}