#include <iostream>
#include <string>

#include "robot/json_loader.hpp"
#include "robot/geometry.hpp"
#include "robot/time_estimator.hpp"
#include "robot/speed_profile.hpp"
#include "robot/export.hpp"
#include "robot/area.hpp"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input.json>\n";
        return 1;
    }

    const std::string jsonFile = argv[1];

    try
    {
        const auto data = robot::loadFromJsonFile(jsonFile);

        robot::SpeedParams params;

        const double width = robot::gadgetWidth(data);
        const double length = robot::pathLength(data.path);
        const double time_s = robot::totalTimeSeconds(data.path, params);

        const double cellSize = 0.01; // 1 cm grid resolution
        const double area_m2 = robot::uniqueCleanedAreaGrid(data.path, width, cellSize);

        std::cout << "Path points: " << data.path.size() << "\n";
        std::cout << "Gadget width: " << width << " m\n";
        std::cout << "Total path length: " << length << " m\n";
        std::cout << "Approx cleaned area: " << area_m2 << " m^2\n";
        std::cout << "Total time: " << time_s << " s\n";
        std::cout << "Grid cell size: " << cellSize << " m\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}