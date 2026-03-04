#pragma once
#include <vector>

namespace robot
{

    struct Vec2
    {
        double x{};
        double y{};
    };

    struct InputData
    {
        std::vector<Vec2> robot; // polygon points (robot frame)
        std::vector<Vec2> path;  // world frame path points
        Vec2 gadget0;            // robot frame
        Vec2 gadget1;            // robot frame
    };

}