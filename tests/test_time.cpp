#include <iostream>
#include <cmath>
#include "robot/time_estimator.hpp"
#include "robot/speed_profile.hpp"
#include "robot/types.hpp"

static bool approx(double a, double b, double eps = 1e-6)
{
    return std::abs(a - b) < eps;
}

int main()
{
    // Straight line: curvature = 0 -> speed = vmax
    // Path: (0,0) -> (2,0), length = 2
    // Time should be 2 / 1.1
    std::vector<robot::Vec2> path{{0, 0}, {2, 0}};
    robot::SpeedParams p;
    const double t = robot::totalTimeSeconds(path, p);

    const double expected = 2.0 / p.vmax;
    if (!approx(t, expected, 1e-6))
    {
        std::cerr << "Time test failed. got " << t << " expected " << expected << "\n";
        return 1;
    }

    std::cout << "Time test passed.\n";
    return 0;
}