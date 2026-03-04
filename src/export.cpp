#include "robot/export.hpp"
#include "robot/curvature.hpp"
#include "robot/speed_profile.hpp"

#include <fstream>
#include <stdexcept>

namespace robot
{

    void exportCurvatureCsv(const std::string &filename,
                            const std::vector<Vec2> &path,
                            const SpeedParams &params)
    {
        std::ofstream out(filename);
        if (!out)
            throw std::runtime_error("Failed to open output file: " + filename);

        const auto k = curvatureProfile(path);

        out << "i,x,y,k,v\n";
        for (size_t i = 0; i < path.size(); ++i)
        {
            const double vi = speedFromCurvature(k[i], params);
            out << i << ","
                << path[i].x << ","
                << path[i].y << ","
                << k[i] << ","
                << vi << "\n";
        }
    }

}