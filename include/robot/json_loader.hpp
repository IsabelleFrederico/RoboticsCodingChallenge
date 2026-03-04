#pragma once
#include "robot/types.hpp"
#include <string>

namespace robot
{

    // Loads the input JSON and returns a filled InputData.
    // Throws std::runtime_error on any error.
    InputData loadFromJsonFile(const std::string &filePath);

}