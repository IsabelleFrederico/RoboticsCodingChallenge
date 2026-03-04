#include "robot/geometry.hpp"
#include <cmath>

namespace robot
{

    double distance(const Vec2 &a, const Vec2 &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double pathLength(const std::vector<Vec2> &path)
    {
        if (path.size() < 2)
            return 0.0;

        double total = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
        {
            total += distance(path[i - 1], path[i]);
        }
        return total;
    }

    double gadgetWidth(const InputData &data)
    {
        return distance(data.gadget0, data.gadget1);
    }

}