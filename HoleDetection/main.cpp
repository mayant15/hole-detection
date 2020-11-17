#include <iostream>

#include "detection.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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

    // The largest of these clusters is probably the exterior boundary, so ignore that
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

    for (int i = 0; i < clusters.size(); ++i)
    {
        if (i == max_idx) continue;

        // Fit circle
        auto model = std::make_shared<pcl::SampleConsensusModelCircle3D<Point>>(clusters[i]);
        auto coefficients = std::make_shared<pcl::ModelCoefficients>();
        auto inliers = std::make_shared<pcl::PointIndices>();
        
        // Create the segmentation object
        pcl::SACSegmentation<Point> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CIRCLE3D);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (clusters[i]);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            return (-1);
        }

        std::cout << "\nHole " << i << ":\n";
        std::cout << "Center: ("<< coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << ")\n";
        std::cout << "Radius: " << coefficients->values[3] << std::endl;
        std::cout << "Normal: ("<< coefficients->values[4] << ", " << coefficients->values[5] << ", " << coefficients->values[6] << ")\n";

        WriteOut(clusters[i], "hole" + std::to_string(i) + ".txt");
    }

    return 0;
}
