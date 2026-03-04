#include "robot/time_estimator.hpp"
#include "robot/geometry.hpp"
#include "robot/curvature.hpp"
#include "robot/speed_profile.hpp"
#include <stdexcept>

namespace robot
{

    double totalTimeSeconds(const std::vector<Vec2> &path, const SpeedParams &params)
    {
        if (path.size() < 2)
            return 0.0;

        // curvature per point (k[i])
        const auto k = curvatureProfile(path);

        // speed per point (v[i])
        std::vector<double> v(path.size(), params.vmax);
        for (size_t i = 0; i < path.size(); ++i)
        {
            v[i] = speedFromCurvature(k[i], params);
        }

        // integrate time per segment
        double totalTime = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            const double ds = distance(path[i], path[i + 1]);
            const double vseg = 0.5 * (v[i] + v[i + 1]);

            if (vseg <= 0.0)
            {
                throw std::runtime_error("Non-positive segment speed encountered");
            }

            totalTime += ds / vseg;
        }

        return totalTime;
    }

}