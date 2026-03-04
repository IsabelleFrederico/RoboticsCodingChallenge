#include "robot/area.hpp"

#include <unordered_set>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <algorithm>

namespace robot
{
    // Pack 2x 32-bit signed indices into 64-bit key safely (works for negatives).
    static uint64_t packCell(int64_t ix, int64_t iy)
    {
        uint64_t ux = static_cast<uint64_t>(static_cast<uint32_t>(ix));
        uint64_t uy = static_cast<uint64_t>(static_cast<uint32_t>(iy));
        return (ux << 32) | uy;
    }

    double uniqueCleanedAreaGrid(const std::vector<Vec2> &path,
                                 double gadgetWidth,
                                 double cellSize)
    {
        if (path.size() < 2)
            return 0.0;
        if (gadgetWidth <= 0.0)
            return 0.0;
        if (cellSize <= 0.0)
            throw std::runtime_error("cellSize must be > 0");

        const double r = 0.5 * gadgetWidth;
        const double r2 = r * r;

        // More robust sampling: don't step too far relative to cell or radius.
        // (Conservative enough to avoid gaps in most practical cases.)
        const double step = 0.5 * std::min(cellSize, r);

        std::unordered_set<uint64_t> visited;
        visited.reserve(100000);

        auto diskIntersectsCell = [&](double x, double y, int64_t ix, int64_t iy) -> bool
        {
            // Cell bounds: [ix*cellSize, (ix+1)*cellSize] x [iy*cellSize, (iy+1)*cellSize]
            const double minX = static_cast<double>(ix) * cellSize;
            const double maxX = static_cast<double>(ix + 1) * cellSize;
            const double minY = static_cast<double>(iy) * cellSize;
            const double maxY = static_cast<double>(iy + 1) * cellSize;

            // Closest point on the rectangle to (x,y)
            const double cx = std::clamp(x, minX, maxX);
            const double cy = std::clamp(y, minY, maxY);

            const double dx = x - cx;
            const double dy = y - cy;
            return (dx * dx + dy * dy) <= r2;
        };

        auto markDiskAt = [&](double x, double y)
        {
            const int64_t cx = static_cast<int64_t>(std::floor(x / cellSize));
            const int64_t cy = static_cast<int64_t>(std::floor(y / cellSize));

            // How many cells to check around the disk center.
            // +1 to be safe at boundaries
            const int64_t dr = static_cast<int64_t>(std::ceil(r / cellSize)) + 1;

            for (int64_t dx = -dr; dx <= dr; ++dx)
            {
                for (int64_t dy = -dr; dy <= dr; ++dy)
                {
                    const int64_t ix = cx + dx;
                    const int64_t iy = cy + dy;

                    if (diskIntersectsCell(x, y, ix, iy))
                    {
                        visited.insert(packCell(ix, iy));
                    }
                }
            }
        };

        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            const Vec2 &a = path[i];
            const Vec2 &b = path[i + 1];

            const double vx = b.x - a.x;
            const double vy = b.y - a.y;
            const double segLen = std::sqrt(vx * vx + vy * vy);

            if (segLen < 1e-15)
            {
                markDiskAt(a.x, a.y);
                continue;
            }

            // If r is extremely small, step could become tiny; still keep sane minimum of 1 sample.
            const int samples = std::max(1, static_cast<int>(std::ceil(segLen / step)));

            for (int s = 0; s <= samples; ++s)
            {
                const double t = static_cast<double>(s) / static_cast<double>(samples);
                markDiskAt(a.x + t * vx, a.y + t * vy);
            }
        }

        return static_cast<double>(visited.size()) * (cellSize * cellSize);
    }

}