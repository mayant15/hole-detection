#include <iostream>

#include "detection.hpp"


void PrintHoleParams(const pcl::ModelCoefficients& coefficients, int i)
{
    std::cout << "\nHole " << i << ":\n";
    std::cout << "Center: ("<< coefficients.values[0] << ", " << coefficients.values[1] << ", " << coefficients.values[2] << ")\n";
    std::cout << "Radius: " << coefficients.values[3] << std::endl;
    std::cout << "Normal: ("<< coefficients.values[4] << ", " << coefficients.values[5] << ", " << coefficients.values[6] << ")\n";
}


int main(int argc, char* argv[])
{
    // Validate arguments
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

    // The largest of these clusters is probably the exterior boundary, so remove that
    int max = clusters[0]->size();
    int max_idx = 0;
    for (int i = 1; i < clusters.size(); ++i)
    {
        if (clusters[i]->size() > max)
        {
            max = clusters[i]->size();
            max_idx = i;
        }
    }
    clusters.erase(clusters.begin() + max_idx);

    // Fit circles to our holes to estimate parameters
    for (int i = 0; i < clusters.size(); ++i)
    {
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;

        FitCircle(clusters[i], inliers, coefficients);

        PrintHoleParams(coefficients, i);
        WriteOut(clusters[i], "hole" + std::to_string(i) + ".txt");
    }

    return 0;
}

