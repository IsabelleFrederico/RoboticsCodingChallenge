#include "robot/curvature.hpp"
#include "robot/geometry.hpp"
#include <cmath>

namespace robot
{

    static double cross2D(const Vec2 &a, const Vec2 &b)
    {
        return a.x * b.y - a.y * b.x;
    }

    double curvatureAt(const std::vector<Vec2> &path, size_t i)
    {
        if (i == 0 || i + 1 >= path.size())
            return 0.0;

        const Vec2 &A = path[i - 1];
        const Vec2 &B = path[i];
        const Vec2 &C = path[i + 1];

        const double AB = distance(A, B);
        const double BC = distance(B, C);
        const double CA = distance(C, A);

        const double eps = 1e-12;
        if (AB < eps || BC < eps || CA < eps)
            return 0.0;

        Vec2 BA{B.x - A.x, B.y - A.y};
        Vec2 CAvec{C.x - A.x, C.y - A.y};

        const double area2 = std::abs(cross2D(BA, CAvec));

        const double k = area2 / (AB * BC * CA);
        return 2.0 * k; // factor 2 for correct curvature formula
    }

    std::vector<double> curvatureProfile(const std::vector<Vec2> &path)
    {
        std::vector<double> result(path.size(), 0.0);

        for (size_t i = 1; i + 1 < path.size(); ++i)
        {
            result[i] = curvatureAt(path, i);
        }

        return result;
    }

}