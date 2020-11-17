#pragma once

#include <memory>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/** ALIASES ***************************************************************/

/** @brief Alias for a 3D point */
using Point = pcl::PointXYZ;

/** @brief Alias for a 3D point cloud */
using PC = pcl::PointCloud<Point>;

/** @brief Alias for a shared pointer to PC */
using PCRef = std::shared_ptr<PC>;

/** @brief Alias for a KdTree */
using KdTree = pcl::KdTreeFLANN<Point>;



/** IO OPERATIONS **********************************************************/

/**
 * @brief Read the provided dataset and create a point cloud
 * 
 * The input file must contain points in the following format:
 * 1. Each line will contain one point
 * 2. Each point will have 3 float coordinates, space separated
 * 
 * For example:
 * 0.1 0.3 3.4
 * 3.3 9.2 1.0
 * 
 * @param  path Path to the file that is to be read 
 * @return A shared pointer to the created point cloud
*/
PCRef ReadIn(const std::string& path)
{
    auto pc = std::make_shared<PC>();

    std::ifstream file(path);

    float x, y, z;
    while (file >> x >> y >> z)
    {
        pc->points.push_back(Point{ x, y, z });
    }

    return pc;
}


/**
 * @brief Write a point cloud to disk. The format followed is same as 
 * the one for the ReadIn function.
 *
 * @param pc   A shared pointer to the point cloud that is to be written
 * @param path Path for the target file
*/
void WriteOut(PCRef pc, const std::string& path)
{
    std::ofstream output(path);
    for (auto p : pc->points)
    {
        output << p.x << " " << p.y << " " << p.z << "\n";
    }
}



/** MATH UTILITIES **********************************************************/

/**
 * @brief Calculate the squared distance between two points in 3D
 * @param p1 Point 1
 * @param p2 Point 2
 * @return float
*/
float SquaredDistance(Point p1, Point p2)
{
    return std::sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}


/**
 * @brief Calculate the coordinates of the centroid of the given points
 * @param pc      A shared pointer to the point cloud under consideration
 * @param indices Indices for the points to consider
 * @return Point
*/
Point Centroid(PCRef pc, const std::vector<int>& indices)
{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    for (auto index : indices)
    {
        auto p = (*pc)[index];
        x += p.x;
        y += p.y;
        z += p.z;
    }

    return Point { x / indices.size(), y / indices.size(), z / indices.size() };
}
