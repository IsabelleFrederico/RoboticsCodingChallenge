#include "robot/speed_profile.hpp"
#include <algorithm>
#include <cmath>

namespace robot
{

    double speedFromCurvature(double k, const SpeedParams &params)
    {
        k = std::abs(k);

        if (k < params.kcrit)
            return params.vmax;

        if (k >= params.kmax)
            return params.vmin;

        const double slope =
            (params.vmax - params.vmin) /
            (params.kmax - params.kcrit);

        const double v =
            params.vmax - slope * (k - params.kcrit);

        return std::clamp(v, params.vmin, params.vmax);
    }

}