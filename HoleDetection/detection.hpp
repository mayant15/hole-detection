#pragma once

#include "util.hpp"


/**
 * @brief Find the neighbors for a point within a specified radius
 * @param search Point under consideration
 * @param radius Radius of the neighborhood
 * @param kdtree A KdTree structure that is to be used for querying
 * @return List of indices for points that are within the specified neighborhood
*/
std::vector<int> RadialNeighbors(Point search, float radius, const KdTree& kdtree)
{
    std::vector<int> indices;
    std::vector<float> sqDistance;
    kdtree.radiusSearch(search, radius, indices, sqDistance);
    return indices;
}


/**
 * @brief Identified points that are at the boundary of the given point cloud
 * 
 * For every point, a spherical neighborhood is considered. If the points within
 * this neighborhood are roughly evenly distributed, then the center is not a 
 * boundary point. Distribution of points is measured by the distance between the
 * center and the centroid.
 * 
 * @param pc        A shared pointer to the point cloud under consideration
 * @param radius    Radius of the neighborhood to consider
 * @param tolerance Tolerance for center-centroid comparison
 * @return A shared reference to a new point cloud containing only the boundary points
*/
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


/**
 * @brief Extract proximity-based clusters from the given point cloud using PCL's
 * built-in Euclidean Cluster Extraction
 * 
 * @param pc             A shared pointer to the point cloud under consideration
 * @param tolerance      Cluster tolerance
 * @param minClusterSize Minimum cluster size
 * @param maxClusterSize Maximum cluster size
 * @return A list of shared pointers to new point clouds, one for each identified cluster
*/
std::vector<PCRef> ExtractEuclideanClusters(PCRef pc, float tolerance = 2.0f, float minClusterSize = 100, float maxClusterSize = 25000)
{
    auto tree = std::make_shared<pcl::search::KdTree<Point>>();
    tree->setInputCloud (pc);

    // Find clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    
    ec.setSearchMethod (tree);
    ec.setInputCloud (pc);
    ec.extract (cluster_indices);


    // Copy to new point clouds
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


/**
 * @brief Fit a 3D circle to a point cloud using PCL's built-in sample consensus based segmentation
 * @param pc                A shared pointer to the point cloud under consideration
 * @param inliers           Reference to an array of point indices that will hold inliers
 * @param coefficients      Reference to a model coefficient structure that will hold the computed coefficients
 * @param distanceThreshold Distance threshold for segmentation
 * @param optimize          Whether coefficient refinement is required
 * @return bool indicating whether the fit was successful
*/
bool FitCircle(PCRef pc, pcl::PointIndices& inliers, pcl::ModelCoefficients& coefficients, float distanceThreshold = 0.01, bool optimize = true)
{
    pcl::SACSegmentation<Point> seg;
    seg.setOptimizeCoefficients (optimize);
    seg.setModelType (pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (pc);
    seg.segment (inliers, coefficients);
}
