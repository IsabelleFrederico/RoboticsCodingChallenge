#include <iostream>
#include <cmath>
#include "robot/geometry.hpp"
#include "robot/types.hpp"

static bool approxEqual(double a, double b, double eps = 1e-9)
{
    return std::abs(a - b) <= eps;
}

int main()
{
    // Test distance: (0,0) to (3,4) should be 5
    robot::Vec2 a{0.0, 0.0};
    robot::Vec2 b{3.0, 4.0};
    const double d = robot::distance(a, b);
    if (!approxEqual(d, 5.0, 1e-9))
    {
        std::cerr << "distance test failed: got " << d << ", expected 5\n";
        return 1;
    }

    // Test pathLength: [(0,0),(3,4)] should be 5
    std::vector<robot::Vec2> path{{0.0, 0.0}, {3.0, 4.0}};
    const double L = robot::pathLength(path);
    if (!approxEqual(L, 5.0, 1e-9))
    {
        std::cerr << "pathLength test failed: got " << L << ", expected 5\n";
        return 1;
    }

    // Test gadgetWidth: gadget from (0,0) to (0,2) => 2
    robot::InputData data;
    data.gadget0 = {0.0, 0.0};
    data.gadget1 = {0.0, 2.0};
    const double w = robot::gadgetWidth(data);
    if (!approxEqual(w, 2.0, 1e-9))
    {
        std::cerr << "gadgetWidth test failed: got " << w << ", expected 2\n";
        return 1;
    }

    std::cout << "All geometry tests passed.\n";
    return 0;
}