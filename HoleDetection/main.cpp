#include <iostream>

#include "detection.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./HoleDetection <file-name>";
        return 0;
    }

    // Read the data and find its boundary
    auto pc = ReadIn(argv[1]);
    auto boundary = ExtractBoundary(pc);
    pc.reset();

    // Get clusters in the boundary
    auto clusters = ExtractEuclideanClusters(boundary);

    int j = 1;
    for (auto cluster : clusters)
    {
        std::string filename = "boundary" + std::to_string(j) + ".txt";
        WriteOut(cluster, filename);
        ++j;
    }

    return 0;
}
