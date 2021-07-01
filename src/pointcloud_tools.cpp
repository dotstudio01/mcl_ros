#include "pointcloud_tools.h"

using namespace std;

pcl::PointCloud<Point>::Ptr load_pointcloud(const string &filename)
{
    if (filename.substr(filename.size() - 3, 3) == "ply")
    {
        // 读取
        pcl::PCLPointCloud2::Ptr cloud2 = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
        pcl::PLYReader reader;
        reader.read(filename, *cloud2);
        // 转换
        pcl::PointCloud<Point>::Ptr cloud;

        cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
        pcl::fromPCLPointCloud2(*cloud2, *cloud);
        return cloud;
    }
    else if (filename.substr(filename.size() - 3, 3) == "pcd")
    {
        // 读取
        pcl::PCLPointCloud2::Ptr cloud2 = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
        pcl::PCDReader reader;
        reader.read(filename, *cloud2);
        // 转换
        pcl::PointCloud<Point>::Ptr cloud;

        cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
        pcl::fromPCLPointCloud2(*cloud2, *cloud);
        return cloud;
    }
    else
    {
        cout << "not supported" << endl;
        return nullptr;
    }
}

pcl::PointCloud<Point>::Ptr remove_outliers(pcl::PointCloud<Point>::Ptr cloud)
{
    pcl::PointCloud<Point>::Ptr cloud_clean = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_clean);
    return cloud_clean;
}

pcl::PointCloud<Point>::Ptr render_heatmap(pcl::PointCloud<Point>::Ptr cloud)
{
    pcl::PointCloud<Point>::Ptr heatmap_cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);

    for (int i = 0; i < cloud->points.size(); i++)
    {
        Point pt = cloud->points[i];
        Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.b = (float)pt.b * 0.7 + (float)colormap_b[pt.a] * 0.3;
        p.g = (float)pt.g * 0.7 + (float)colormap_g[pt.a] * 0.3;
        p.r = (float)pt.r * 0.7 + (float)colormap_r[pt.a] * 0.3;
        p.a = pt.a;
        heatmap_cloud->push_back(p);
    }
    return heatmap_cloud;
}

void save_pointcloud(const string &filename, pcl::PointCloud<Point>::Ptr cloud)
{
    if (filename.substr(filename.size() - 3, 3) == "ply")
    {
        pcl::PLYWriter writer;
        writer.write(filename, *cloud);
    }
    else if (filename.substr(filename.size() - 3, 3) == "pcd")
    {
        pcl::PCDWriter writer;
        writer.write(filename, *cloud);
    }
    else
    {
        cout << "not supported" << endl;
        return;
    }
}

pcl::PointCloud<Point>::Ptr voxel_filter(pcl::PointCloud<Point>::Ptr cloud)
{
    pcl::PointCloud<Point>::Ptr cloud_filtered = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::VoxelGrid<Point> grid;
    grid.setLeafSize(1, 1, 1);
    grid.setInputCloud(cloud);
    grid.filter(*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<Point>::Ptr importance_filter(pcl::PointCloud<Point>::Ptr cloud, double threshold)
{
    // 计算点云的范围
    Eigen::Vector3f min_p, max_p;
    min_p.setConstant(FLT_MAX);
    max_p.setConstant(FLT_MIN);
    pcl::PointCloud<Point>::Ptr new_cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Point *p = &(cloud->points[i]);
        if(p->a < 64)
            continue;
        new_cloud->push_back(*p);
        for (int d = 0; d < 3; d++)
        {
            min_p(d) = min_p(d) < p->data[d] ? min_p(d) : p->data[d];
            max_p(d) = max_p(d) > p->data[d] ? max_p(d) : p->data[d];
        }
    }
    cloud = new_cloud;
    Eigen::Vector3f leaf_size;
    leaf_size << 0.1, 0.1, 0.1;
    Eigen::Vector3i dims;
    for (int i = 0; i < 3; i++)
    {
        dims(i) = (int)((max_p(i) - min_p(i)) / leaf_size(i)) + 1;
    }
    // 保存当前格子中最重要的点
    map<uint64_t, Point *> point_buffer;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Point *p = &(cloud->points[i]);
        uint64_t idx =
            ((uint64_t)((p->x - min_p(0)) / leaf_size(0))) << 40 + ((uint64_t)((p->y - min_p(1)) / leaf_size(1))) << 20 + ((uint64_t)((p->z - min_p(2)) / leaf_size(2)));
        if (point_buffer.find(idx) != point_buffer.end())
        {
            if (p->a > point_buffer[idx]->a)
            {
                point_buffer[idx] = p;
            }
        }
        else
        {
            point_buffer[idx] = p;
        }
    }
    // 保存裁剪过后的点云（每个体素内保留alpha最大的，区别于体素滤波求平均的操作）
    pcl::PointCloud<Point>::Ptr simplified_cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    // 保存极大值点构成的点云
    pcl::PointCloud<Point>::Ptr peak_cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    // 保存极大值点附近，响应大于alpha*threshold的点
    pcl::PointCloud<Point>::Ptr cloud_selected = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    // 保存滤波过后的点云
    pcl::PointCloud<Point>::Ptr cloud_filtered = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    for (map<uint64_t, Point *>::iterator it = point_buffer.begin(); it != point_buffer.end(); it++)
    {
        simplified_cloud->push_back(Point(*it->second));
    }
    pcl::KdTreeFLANN<Point, flann::L2_Simple<float>> kdtree;
    // 根据简化后的点云，建立peak点云
    kdtree.setInputCloud(simplified_cloud);
    int k = 3;
    vector<int> point_idx;
    vector<float> point_squared_distance;
    point_idx.resize(k);
    point_squared_distance.resize(k);
    for (int i = 0; i < simplified_cloud->points.size(); i++)
    {
        Point *pt = &(simplified_cloud->points[i]);
        if (kdtree.nearestKSearch(*pt, k, point_idx, point_squared_distance))
        {
            bool isPeak = true;
            for (int j = 0; j < point_idx.size(); j++)
            {
                if (kdtree.getInputCloud()->points[point_idx[j]].a > pt->a)
                {
                    isPeak = false;
                    break;
                }
            }
            if (isPeak)
            {
                peak_cloud->push_back(Point(*pt));
            }
        }
    }
    // 根据peak_cloud，计算peak_cloud中某个点附近，响应值大于阈值的点
    kdtree = pcl::KdTreeFLANN<Point, flann::L2_Simple<float>>();
    kdtree.setInputCloud(peak_cloud);
    k = 1;
    point_idx.resize(k);
    point_squared_distance.resize(k);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Point *pt = &(cloud->points[i]);
        
        if (kdtree.nearestKSearch(*pt, k, point_idx, point_squared_distance))
        {
            if ((double)pt->a > (double)(peak_cloud->points[point_idx[0]].a) * threshold)
            {
                cloud_selected->push_back(Point(*pt));
            }
        }
    }
    long long pt_cnt[256];
    memset(pt_cnt, 0, sizeof(pt_cnt));
    
    // 取所有cloud_selected周围的点，作为地图中的参考点
    kdtree = pcl::KdTreeFLANN<Point, flann::L2_Simple<float>>();
    kdtree.setInputCloud(cloud_selected);
    k = 1;
    point_idx.resize(k);
    point_squared_distance.resize(k);
    for(int i =0; i < cloud->points.size(); i++)
    {
        Point *pt = &(cloud->points[i]);
        if (kdtree.nearestKSearch(*pt, k, point_idx, point_squared_distance))
        {
            pt_cnt[pt->a]++;
            if(point_squared_distance[0] < 0.1)
            {
                cloud_filtered->push_back(Point(*pt));
            }
        }
    }
    cout << "pt_cnt" << endl;
    for (int i = 0; i < 256; i++)
    {
        cout << pt_cnt[i] << " ";
    }
    cout << endl;
    return cloud_filtered;
}

void ignore_alpha_channel(pcl::PointCloud<Point>::Ptr cloud)
{
    for (int i = 0; i < cloud->points.size(); i++)
    {
        cloud->points[i].a = 255;
    }
}

double get_double(string s, int &cursor)
{
    string num_s;
    char data;
    data = s[cursor];
    while (!(data == '+' || data == '-' || data == '.' || (data >= '0' && data <= '9')))
    {
        cursor++;
        data = s[cursor];
    }
    while ((data == '+' || data == '-' || data == '.' || data == 'e' || (data >= '0' && data <= '9')))
    {
        num_s = num_s + data;
        cursor++;
        if (cursor >= s.size())
            break;
        data = s[cursor];
    }
    return atof(num_s.c_str());
}

pcl::PointCloud<Point>::Ptr perform_transform(pcl::PointCloud<Point>::Ptr cloud, string transform_filename)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double s;
    std::ifstream fs(transform_filename);
    int cursor = 0;
    string line;
    for (int i = 0; i < 11; i++)
    {
        getline(fs, line);
    }
    for (int i = 0; i < 3; i++)
    {
        getline(fs, line);
        cursor = 0;
        for (int j = 0; j < 3; j++)
        {
            R(i, j) = get_double(line, cursor);
        }
    }
    getline(fs, line);
    getline(fs, line);
    cursor = 0;
    for (int i = 0; i < 3; i++)
    {
        t(i) = get_double(line, cursor);
    }
    getline(fs, line);
    cursor = 0;
    s = get_double(line, cursor);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    cout << T << endl;
    cout <<"Scale: " << s << endl;
    Eigen::Matrix3d R_inv = R.transpose();
    pcl::PointCloud<Point>::Ptr cloud_transformed = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::copyPointCloud(*cloud, *cloud_transformed);
    for(int i = 0; i< cloud_transformed->points.size(); i++)
    {
        Eigen::Vector3d pt;
        pt(0) = cloud_transformed->points[i].x;
        pt(1) = cloud_transformed->points[i].y;
        pt(2) = cloud_transformed->points[i].z;
        pt = R_inv * (pt - t) / s;
        cloud_transformed->points[i].x = pt(0);
        cloud_transformed->points[i].y = pt(1);
        cloud_transformed->points[i].z = pt(2);
    }
    return cloud_transformed;
}

void process_pointcloud(const string &folder)
{
    pcl::PointCloud<Point>::Ptr cloud;
    pcl::PointCloud<Point>::Ptr cloud_transformed = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr cloud_clean;
    pcl::PointCloud<Point>::Ptr cloud_heatmap;
    pcl::PointCloud<Point>::Ptr cloud_filtered;
    pcl::PointCloud<Point>::Ptr cloud_heatmap_filtered;
    pcl::PointCloud<Point>::Ptr cloud_merged = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>);
    boost::format file_fmt(folder + "/%s_%03d.%s");
    int i = 0;
    while (true)
    {
        string filename = (file_fmt % "pointcloud" % i % "ply").str();
        if (!file_exists(filename))
            break;
        cout << "loading point cloud " << filename << endl;

        cloud = load_pointcloud(filename);
        cout << "\nraw count " << cloud->points.size() << endl;

        string report_filename = (file_fmt % "report" % i % "txt").str();
        if (file_exists(report_filename))
        {
            cout << "calibration file detected" << endl;
            cloud_transformed = perform_transform(cloud, report_filename);
            save_pointcloud((file_fmt % "cloud_transformed" % i % "pcd").str(), cloud_transformed);
            cloud = cloud_transformed;
        }
        cloud_clean = remove_outliers(cloud);
        *cloud_merged += *cloud_clean;
        cout << "\nclean count " << cloud_clean->points.size() << endl;

#ifdef SAVE_DEBUG_FILES
        cloud_heatmap = render_heatmap(cloud_clean);

        cloud_filtered = importance_filter(cloud_clean, 0.85);
        cloud_heatmap_filtered = importance_filter(cloud_heatmap, 0.85);
        cout << "\nfiltered count " << cloud_filtered->points.size() << endl;
        // ignore_alpha_channel(cloud_clean);
        // ignore_alpha_channel(cloud_heatmap);
        // ignore_alpha_channel(cloud_filtered);
        // ignore_alpha_channel(cloud_heatmap_filtered);
        save_pointcloud((file_fmt % "cloud_clean" % i % "pcd").str(), cloud_clean);
        save_pointcloud((file_fmt % "cloud_heatmap" % i % "pcd").str(), cloud_heatmap);
        save_pointcloud((file_fmt % "cloud_filtered" % i % "pcd").str(), cloud_filtered);
        save_pointcloud((file_fmt % "cloud_heatmap_filtered" % i % "pcd").str(), cloud_heatmap_filtered);
#endif
        i++;
    }
    pcl::PointCloud<Point>::Ptr map;
    pcl::PointCloud<Point>::Ptr map_filtered;
    pcl::PointCloud<Point>::Ptr map_visualize;

    map = cloud_merged;
    map_filtered = importance_filter(map, 0.75);
    map_visualize = voxel_filter(map);

    cout << "total points: " << map->points.size() << endl;
    cout << "filtered points: " << map_filtered->points.size() << endl;
    cout << "visualize points: " << map_visualize->points.size() << endl;

    save_pointcloud(folder + "/map.pcd", map);
    save_pointcloud(folder + "/map_filtered.pcd", map_filtered);
    save_pointcloud(folder + "/map_visualize.pcd", map_visualize);
}

void build_map(const string &folder)
{
    vector<cv::Mat> colorImgs, depthImgs, heatmapImgs;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses; // 相机位姿
    bool heatmap_exists = false;

    ifstream fin(folder + "/groundtruth.txt");
    int start_idx = 3;
    int end_idx = 503;
    if (!fin)
    {
        cout << "pose file not found";
        return;
    }
    string s;
    for (int i = 0; i < start_idx; i++)
        getline(fin, s);
    for (int i = 0; i < end_idx - start_idx; i++)
    {
        string stamp;
        fin >> stamp;
        boost::format fmt(folder + "/%s/%s.png"); //图像文件格式
        colorImgs.push_back(cv::imread((fmt % "rgb" % stamp).str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % stamp).str(), -1)); // 使用-1读取原始图像
        // 如果第一张图片有heatmap，认为heatmap存在
        if (i == 0)
            heatmap_exists = file_exists((fmt % "heatmap" % stamp).str());
        if (heatmap_exists)
        {
            heatmap_exists = true;
            cv::Mat heatmapImg = cv::imread((fmt % "heatmap" % stamp).str(), -1);
            cv::resize(heatmapImg, heatmapImg, colorImgs[0].size());
            heatmapImgs.push_back(heatmapImg);
        }

        double data[7] = {0};
        for (auto &d : data)
            fin >> d;
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 计算点云并拼接
    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    cx = 320.0;
    cy = 240.0;
    fx = 554.3;
    fy = 554.3;
    depthScale = 5000.0;

    cout << "正在将图像转换为点云..." << endl;

    Eigen::Matrix3d R;
    R << 0, 0, 1,
        -1, 0, 0,
        0, -1, 0;
    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < end_idx - start_idx; i += 1)
    {
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        cv::Mat heatmap;
        if (heatmap_exists)
            heatmap = heatmapImgs[i];
        Eigen::Isometry3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0)
                    continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * (R * point);

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];

                p.b = color.data[v * color.step + u * color.channels()] / 2 + colormap_b[heatmap.ptr<uint8_t>(v)[u]] / 2;
                p.g = color.data[v * color.step + u * color.channels() + 1] / 2 + colormap_g[heatmap.ptr<uint8_t>(v)[u]] / 2;
                p.r = color.data[v * color.step + u * color.channels() + 2] / 2 + colormap_r[heatmap.ptr<uint8_t>(v)[u]] / 2;
                p.a = 255;
                if (heatmap_exists)
                {
                    p.a = heatmap.ptr<uint8_t>(v)[u];
                }
                pointCloud->points.push_back(p);
            }
    }
    PointT new_pt;
    new_pt.x = 50;
    new_pt.y = 0;
    new_pt.z = 0;
    new_pt.b = 0;
    new_pt.g = 0;
    new_pt.r = 255;
    new_pt.a = 255;
    pointCloud->push_back(new_pt);
    new_pt.x = 0;
    new_pt.y = 50;
    new_pt.g = 255;
    new_pt.r = 0;
    new_pt.a = 255;
    pointCloud->push_back(new_pt);

    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(0.02, 0.02, 0.02);
    grid.setInputCloud(pointCloud);
    grid.filter(*pointCloud);
    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("usage:./pointcloud_tools [command] [data_folder]");
        return -1;
    }
    string command = argv[1];
    string folder = argv[2];
    if (command == "process_pointcloud")
    {
        process_pointcloud(folder);
    }
    else if (command == "build_map")
    {
        build_map(folder);
    }
    else
    {
        printf("unsupported command\n");
    }
    return 0;
}