#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include "robot/area.hpp"
#include "robot/types.hpp"

static bool approx(double a, double b, double rel = 0.05)
{
    // relative tolerance
    const double denom = std::max(1e-12, std::abs(b));
    return (std::abs(a - b) / denom) < rel;
}

static std::vector<robot::Vec2> translate(const std::vector<robot::Vec2> &p, double tx, double ty)
{
    std::vector<robot::Vec2> out;
    out.reserve(p.size());
    for (const auto &v : p)
        out.push_back(robot::Vec2{v.x + tx, v.y + ty});
    return out;
}

int main()
{
    const double w = 0.30; // gadget width (diameter)
    const double cell = 0.01;

    // 1) Single pass line
    std::vector<robot::Vec2> p1{{0, 0}, {1, 0}};

    // 2) Same line forward and back (overlap)
    std::vector<robot::Vec2> p2{{0, 0}, {1, 0}, {0, 0}};

    const double a1 = robot::uniqueCleanedAreaGrid(p1, w, cell);
    const double a2 = robot::uniqueCleanedAreaGrid(p2, w, cell);

    if (!approx(a2, a1, 0.05))
    {
        std::cerr << "Overlap test failed: a1=" << a1 << " a2=" << a2 << "\n";
        return 1;
    }

    // 3) Idempotence: repeat same path points shouldn't change result (still same swept area)
    std::vector<robot::Vec2> p3{{0, 0}, {1, 0}, {1, 0}, {1, 0}};
    const double a3 = robot::uniqueCleanedAreaGrid(p3, w, cell);

    if (!approx(a3, a1, 0.05))
    {
        std::cerr << "Idempotence test failed: a1=" << a1 << " a3=" << a3 << "\n";
        return 1;
    }

    // 4) Translation invariance: shifting the whole path should not change area
    auto p1_shift = translate(p1, 10.0, -7.5);
    const double a1_shift = robot::uniqueCleanedAreaGrid(p1_shift, w, cell);

    if (!approx(a1_shift, a1, 0.05))
    {
        std::cerr << "Translation test failed: a1=" << a1 << " a1_shift=" << a1_shift << "\n";
        return 1;
    }

    // 5) Monotonicity: larger width should clean >= area
    const double w2 = 0.40;
    const double a_w2 = robot::uniqueCleanedAreaGrid(p1, w2, cell);

    if (a_w2 + 1e-12 < a1) // allow tiny numeric wiggle
    {
        std::cerr << "Monotonicity test failed: a(w=" << w << ")=" << a1
                  << " a(w=" << w2 << ")=" << a_w2 << "\n";
        return 1;
    }

    std::cout << "All area tests passed.\n"
              << "a1=" << a1 << " a2=" << a2 << " a3=" << a3
              << " a1_shift=" << a1_shift << " a_w2=" << a_w2 << "\n";

    return 0;
}