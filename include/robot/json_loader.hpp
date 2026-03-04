#pragma once
#include "robot/types.hpp"
#include <string>

namespace robot
{

    InputData loadFromJsonFile(const std::string &filePath);

}