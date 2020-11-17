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

PCRef ExtractBoundary(PCRef pc, float radius = 2.0f, float tolerance = 0.55f)
{
    KdTree kdtree;
    kdtree.setInputCloud(pc);

    std::vector<int> bdr;
    for (int i = 0; i < pc->size(); ++i)
    {
        auto p = (*pc)[i];
        auto res = RadialNeighbors(p, radius, kdtree);
        Point c = Centroid(pc, res);
        if (SquaredDistance(c, p) > tolerance)
        {
            bdr.push_back(i);
        }
    }

    auto cloud = std::make_shared<PC>();
    pcl::copyPointCloud(*pc, bdr, *cloud);

    return cloud;
}

std::vector<PCRef> ExtractEuclideanClusters(PCRef pc, float tolerance = 2.0f, float minClusterSize = 100, float maxClusterSize = 25000)
{
    auto tree = std::make_shared<pcl::search::KdTree<Point>>();
    tree->setInputCloud (pc);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    
    ec.setSearchMethod (tree);
    ec.setInputCloud (pc);
    ec.extract (cluster_indices);

    std::vector<PCRef> clusters;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        auto cluster = std::make_shared<PC>();
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cluster->push_back((*pc)[*pit]);
        }

        cluster->width = cluster->size ();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    return clusters;
}
