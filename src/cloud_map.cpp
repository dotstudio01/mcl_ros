#include "cloud_map.h"

using namespace std;
CloudMap::CloudMap()
{
}

void CloudMap::init_points()
{
    int point_count = cloud->points.size();
    this->points = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, point_count);
    for (int i = 0; i < point_count; i++)
    {
        this->points(0, i) = cloud->points[i].x;
        this->points(1, i) = cloud->points[i].y;
        this->points(2, i) = cloud->points[i].z;
    }
    this->pixels = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, point_count);
}

void CloudMap::Load_PLY(const std::string &file_name)
{
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
    pcl::PLYReader reader;
    reader.read(file_name, *cloud2);

    cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);
    init_points();
}

void CloudMap::Load_PCD(const std::string &file_name)
{
    cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr cloud_tmp = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::PCDReader reader;
    reader.read(file_name, *cloud_tmp);
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(cloud_tmp);
    voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    voxel_grid.filter(*cloud);
    init_points();
}

void CloudMap::Export_PCD(const std::string &file_name)
{
    pcl::PCDWriter writer;
    writer.write(file_name, *cloud);
}

vector<Image> CloudMap::project(CameraIntrinsics K, PoseMatrix poseMat, int height, int width)
{
    int point_count = points.cols();
    // 将点云数据从世界坐标系投影到相机坐标系
    Eigen::Matrix<double, 3, 3> rot = poseMat.block<3, 3>(0, 0);
    Eigen::Matrix<double, 3, 1> trans = poseMat.block<3, 1>(0, 3);

    for (int i = 0; i < point_count; i++)
        pixels.block<3, 1>(0, i) = points.block<3, 1>(0, i) - trans;
    pixels = K * rot.transpose() * pixels;
    for (int i = 0; i < point_count; i++)
    {
        pixels.block<2, 1>(0, i) /= pixels(2, i);
    }
    cv::Mat color = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat depth = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat label = cv::Mat::zeros(height, width, CV_8UC1);
    depth = depth + 999;
    int u, v;
    for (int i = 0; i < point_count; i++)
    {
        if(pixels(2, i) > 50)
            continue;
        u = pixels(0, i) + 0.5;
        v = pixels(1, i) + 0.5;
        if ((u >= 0) && (u < width) && (v >= 0) && (v < height))
        {
            if ((depth.ptr<double>(v)[u] > pixels(2, i)) && (pixels(2, i) > 0))
            {
                depth.ptr<double>(v)[u] = pixels(2, i);
                // 预置points，可能可以加速
                color.ptr<uchar>(v)[u * 3] = cloud->points[i].b;
                color.ptr<uchar>(v)[u * 3 + 1] = cloud->points[i].g;
                color.ptr<uchar>(v)[u * 3 + 2] = cloud->points[i].r;
                label.ptr<uchar>(v)[u] = (uchar)cloud->points[i].label;
            }
        }
    }
    vector<Image> images;
    images.push_back(color);
    images.push_back(label);
    return images;
}

vector<Image> CloudMap::project(CameraIntrinsics K, Pose pose, int height, int width)
{
    // 将位姿转换为位姿矩阵
    PoseMatrix poseMat = PoseMatrix::Identity();
    double x, y, z, theta;
    x = pose(0);
    y = pose(1);
    z = pose(2);
    theta = pose(3);
    poseMat << cos(theta), 0, -sin(theta), x,
        0, 1, 0, y,
        sin(theta), 0, cos(theta), z,
        0, 0, 0, 1;
    return project(K, poseMat, height, width);
}

/*
double CloudMap::compute_fitness(CameraIntrinsics K, Image image, Pose pose)
{
    int height = image.rows;
    int width = image.cols;
    vector<Image> images = project(K, pose, height, width);
    Image image_reprojected = images[0];
    double tolerance = 10;
    int matchCount = 0;
    int validTotal = 0;
    double matchError = 0;
    for (int u = 0; u < image.cols; u++)
    {
        for (int v = 0; v < image.rows; v++)
        {
            if (image_reprojected.ptr<uchar>(v)[u * 3 + 0] + image_reprojected.ptr<uchar>(v)[u * 3 + 1] + image_reprojected.ptr<uchar>(v)[u * 3 + 2] > 0)
            {
                double error = 0;
                error += pow(image_reprojected.ptr<uchar>(v)[u * 3 + 0] - image.ptr<uchar>(v)[u * 3 + 0], 2);
                error += pow(image_reprojected.ptr<uchar>(v)[u * 3 + 1] - image.ptr<uchar>(v)[u * 3 + 1], 2);
                error += pow(image_reprojected.ptr<uchar>(v)[u * 3 + 2] - image.ptr<uchar>(v)[u * 3 + 2], 2);
                error = sqrt(error);
                validTotal++;
                if (error < tolerance)
                {
                    matchCount++;
                    matchError += error;
                }
            }
        }
    }
    double fitness = 0;
    if (validTotal > 0)
    {
        if ((double)matchCount / (double)(image.cols * image.rows) > 0.1)
        {
            fitness = (double)matchCount / (double)(validTotal);
        }
        else
        {
            fitness = (double)matchCount / (double)(image.cols * image.rows);
        }
        
    }
    else
    {
        fitness = 1e-6;
    }
    return fitness;
}
*/

// 计算fitnness的时候，需要保证标签一致
double CloudMap::compute_fitness(CameraIntrinsics K, Image image, Image label, Pose pose)
{
    int height = image.rows;
    int width = image.cols;
    vector<Image> images = project(K, pose, height, width);
    Image image_reprojected = images[0];
    Image label_reprojected = images[1];
    double tolerance = 10;
    int matchCount = 0;
    int validTotal = 0;
    double matchError = 0;
    for (int u = 0; u < image.cols; u++)
    {
        for (int v = 0; v < image.rows; v++)
        {
            if (image_reprojected.ptr<uchar>(v)[u * 3 + 0] + image_reprojected.ptr<uchar>(v)[u * 3 + 1] + image_reprojected.ptr<uchar>(v)[u * 3 + 2] > 0)
            {
                double error = 0;
                error += pow(image_reprojected.ptr<uchar>(v)[u * 3 + 0] - image.ptr<uchar>(v)[u * 3 + 0], 2);
                error += pow(image_reprojected.ptr<uchar>(v)[u * 3 + 1] - image.ptr<uchar>(v)[u * 3 + 1], 2);
                error += pow(image_reprojected.ptr<uchar>(v)[u * 3 + 2] - image.ptr<uchar>(v)[u * 3 + 2], 2);
                error = sqrt(error);
                validTotal++;
                // if ((error < tolerance) && (label_reprojected.ptr<uchar>(v)[u] == label.ptr<uchar>(v)[u]))
                if(error < tolerance)
                {
                    matchCount++;
                    matchError += error;
                }
            }
        }
    }
    double fitness = 0;
    if (validTotal > 0)
    {
        if ((double)matchCount / (double)(image.cols * image.rows) > 0.1)
        {
            fitness = (double)matchCount / (double)(validTotal);
        }
        else
        {
            fitness = (double)matchCount / (double)(image.cols * image.rows);
        }
    }
    else
    {
        fitness = 1e-6;
    }
    return fitness;
}