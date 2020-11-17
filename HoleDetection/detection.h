#pragma once

#include "util.h"

std::vector<int> RadialNeighbors(Point search, float radius, const KdTree& kdtree)
{
    std::vector<int> indices;
    std::vector<float> sqDistance;

    if (kdtree.radiusSearch(search, radius, indices, sqDistance) <= 0)
    {
        std::cout << "ERROR (" << __LINE__ << "): Radial search failed.\n";
    }

    return indices;
}
