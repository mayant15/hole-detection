#include <iostream>

#include "detection.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./HoleDetection <file-name>";
        return 0;
    }

    auto pc = ReadIn(argv[1]);

    KdTree kdtree;
    kdtree.setInputCloud(pc);

    std::vector<int> bdr;
    for (int i = 0; i < pc->size(); ++i)
    {
        auto p = (*pc)[i];
        auto res = RadialNeighbors(p, 2.0f, kdtree);
        Point c = Centroid(pc, res);
        if (SquaredDistance(c, p) > 0.5)
        {
            bdr.push_back(i);
        }
    }

    WriteOut(pc, bdr, "boundary.txt");

    return 0;
}
