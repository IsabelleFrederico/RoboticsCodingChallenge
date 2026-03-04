#pragma once

namespace robot
{

    struct SpeedParams
    {
        double vmax{1.1};
        double vmin{0.15};
        double kcrit{0.5};
        double kmax{10.0};
    };

    double speedFromCurvature(double k, const SpeedParams &params);

}