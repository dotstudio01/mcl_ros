#ifndef __CLOUD_MAP_H
#define __CLOUD_MAP_H
#include "types.h"
#include <string>
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/filters/voxel_grid.h"
#include "stdio.h"
#include <boost/shared_ptr.hpp>
#include <map>


typedef pcl::PointXYZRGBL Point;
class CloudMap
{
public:
    typedef boost::shared_ptr<CloudMap> Ptr;
    CloudMap();
    void Load_PLY(const std::string &file_name);
    void Load_PCD(const std::string &file_name);
    void Export_PCD(const std::string &file_name);
    void init_points();
    std::vector<Image> project(CameraIntrinsics K, Pose pose, int height, int width);
    std::vector<Image> project(CameraIntrinsics K, PoseMatrix poseMat, int height, int width);
    // double compute_fitness(CameraIntrinsics K, Image image, Pose pose);
    double compute_fitness(CameraIntrinsics K, Image image, Image label, Pose pose);
    inline size_t point_count() { return cloud->points.size(); };
    pcl::PointCloud<Point>::Ptr cloud;

private:
    Eigen::Matrix<double, 3, Eigen::Dynamic> points;
    // buffer, avoid memory allocation each time
    Eigen::Matrix<double, 3, Eigen::Dynamic> pixels;
};
#endif