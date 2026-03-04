#pragma once

#include <string>
#include <vector>
#include <cstddef>

#include "robot/types.hpp"
#include "robot/speed_profile.hpp"

namespace robot
{

    void exportCurvatureCsv(const std::string &filename,
                            const std::vector<Vec2> &path,
                            const SpeedParams &params);

}