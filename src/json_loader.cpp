#include "robot/json_loader.hpp"
#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

namespace robot
{

    static Vec2 vec2FromJsonArray(const json &a, const char *name)
    {
        if (!a.is_array() || a.size() != 2 || !a[0].is_number() || !a[1].is_number())
        {
            throw std::runtime_error(std::string(name) + " must be [number, number]");
        }
        return Vec2{a[0].get<double>(), a[1].get<double>()};
    }

    InputData loadFromJsonFile(const std::string &filePath)
    {
        std::ifstream file(filePath);
        if (!file)
        {
            throw std::runtime_error("Failed to open JSON file: " + filePath);
        }

        json j;
        file >> j;

        InputData data;

        // robot polygon
        if (!j.contains("robot") || !j["robot"].is_array())
        {
            throw std::runtime_error("Missing array: robot");
        }
        for (const auto &p : j["robot"])
        {
            data.robot.push_back(vec2FromJsonArray(p, "robot[i]"));
        }

        // cleaning gadget line
        if (!j.contains("cleaning_gadget") || !j["cleaning_gadget"].is_array())
        {
            throw std::runtime_error("Missing array: cleaning_gadget");
        }
        const auto &cg = j["cleaning_gadget"];
        if (cg.size() != 2)
        {
            throw std::runtime_error("cleaning_gadget must have exactly 2 points");
        }
        data.gadget0 = vec2FromJsonArray(cg[0], "cleaning_gadget[0]");
        data.gadget1 = vec2FromJsonArray(cg[1], "cleaning_gadget[1]");

        // path
        if (!j.contains("path") || !j["path"].is_array())
        {
            throw std::runtime_error("Missing array: path");
        }
        for (const auto &p : j["path"])
        {
            data.path.push_back(vec2FromJsonArray(p, "path[i]"));
        }
        if (data.path.size() < 2)
        {
            throw std::runtime_error("path must have at least 2 points");
        }

        return data;
    }

}