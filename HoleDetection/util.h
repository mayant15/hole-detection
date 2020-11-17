#pragma once

#include <memory>
#include <fstream>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

// Aliases

using Point = pcl::PointXYZ;
using PC = pcl::PointCloud<Point>;
using PCRef = std::shared_ptr<PC>;
using KdTree = pcl::KdTreeFLANN<Point>;
using KResult = std::vector<std::pair<int, float>>;

// IO Operations

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

void WriteOut(PCRef pc, const std::vector<int>& indices, const std::string& path)
{
    std::ofstream output(path);
    for (auto idx : indices)
    {
        auto p = (*pc)[idx];
        output << p.x << " " << p.y << " " << p.z << "\n";
    }
}

void WriteOut(PCRef pc, const std::string& path)
{
    std::ofstream output(path);
    for (auto p : pc->points)
    {
        output << p.x << " " << p.y << " " << p.z << "\n";
    }
}

// Math Functions

float SquaredDistance(Point p1, Point p2)
{
    return std::sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
}

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
