cmake_minimum_required(VERSION 3.20)
project(robot_path_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(robot_path_project
    src/main.cpp
    src/geometry.cpp
    src/json_loader.cpp
    src/curvature.cpp
    src/speed_profile.cpp
    src/time_estimator.cpp
    src/export.cpp
    src/area.cpp
)

target_include_directories(robot_path_project PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party
)


# --- Tests ---

enable_testing()

add_executable(test_geometry
    tests/test_geometry.cpp
    src/geometry.cpp
)
target_include_directories(test_geometry PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

add_test(NAME test_geometry COMMAND test_geometry)

add_executable(test_curvature
    tests/test_curvature.cpp
    src/geometry.cpp
    src/curvature.cpp
    src/speed_profile.cpp
)
target_include_directories(test_curvature PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

add_test(NAME test_curvature COMMAND test_curvature)

add_executable(test_speed
    tests/test_speed.cpp
    src/speed_profile.cpp
)

target_include_directories(test_speed PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

add_test(NAME test_speed COMMAND test_speed)

add_executable(test_time
    tests/test_time.cpp
    src/geometry.cpp
    src/curvature.cpp
    src/speed_profile.cpp
    src/time_estimator.cpp
    src/area.cpp
)
target_include_directories(test_time PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
add_test(NAME test_time COMMAND test_time)

add_executable(test_area
    tests/test_area.cpp
    src/area.cpp
)
target_include_directories(test_area PRIVATE ${CMAKE_SOURCE_DIR}/include)
add_test(NAME test_area COMMAND test_area)
