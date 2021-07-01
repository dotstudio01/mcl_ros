#include "mcl.h"

using namespace std;
using namespace ros;

NodeHandle *node;
Publisher map_pub;
Publisher particle_cloud_pub;
Publisher path_es_pub;
Publisher path_gt_pub;
Publisher pose_es_pub;
Publisher pose_gt_pub;
Publisher reprojected_es_pub;
Publisher reprojected_gt_pub;
ParticleFilter pf;

CloudMap::Ptr cloud_map;
pcl::PointCloud<Point>::Ptr particle_cloud;
CameraIntrinsics K;
PoseVector poses_gt;

void PublishPointCloud(pcl::PointCloud<Point>::Ptr cloud, Publisher publisher)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    publisher.publish(msg);
}

void PublishImage(Image image, Publisher publisher)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = "world";
    publisher.publish(*msg);
}

void PublishPose(Pose pose, Publisher publisher)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = pose(0);
    msg.pose.position.y = pose(1);
    msg.pose.position.z = pose(2);
    msg.pose.orientation.w = cos(pose(3) / 2.0);
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = sin(pose(3) / 2.0);
    msg.pose.orientation.z = 0;
    publisher.publish(msg);
}

void PublishPath(PoseVector poses, Publisher publisher)
{
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    for (int i = 0; i < poses.size(); i++)
    {
        Pose pose = poses[i];
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now() - ros::Duration(poses.size() - i);
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        pose_msg.pose.position.z = pose(2);
        pose_msg.pose.orientation.w = cos(pose(3));
        pose_msg.pose.orientation.x = 0;
        pose_msg.pose.orientation.y = sin(pose(3));
        pose_msg.pose.orientation.z = 0;
        msg.poses.push_back(pose_msg);
    }
    publisher.publish(msg);
}

void init_particle_filter()
{
    srand(100);
    int particle_count = 800;
    double xmin = -5;
    double xmax = 5;
    double ymin = -5;
    double ymax = 5;
    double zmin = -5;
    double zmax = 5;
    double std_trans = 0.1;
    double std_rot = 5 * M_PI / 180.0;
    // 设置粒子滤波参数，初始化
    pf.set_param(std_trans, std_rot);
    pf.set_map(cloud_map);
    pf.set_camera_param(K);
    pf.init(particle_count, xmin, xmax, ymin, ymax, zmin, zmax);
}

int main(int argc, char **argv)
{
    init(argc, argv, "MonteCarloLocalizer");

    node = new NodeHandle();
    map_pub = node->advertise<sensor_msgs::PointCloud2>("cloud_map", 5);
    particle_cloud_pub = node->advertise<sensor_msgs::PointCloud2>("particle_map", 5);
    pose_es_pub = node->advertise<geometry_msgs::PoseStamped>("es_pose", 5);
    pose_gt_pub = node->advertise<geometry_msgs::PoseStamped>("gt_pose", 5);
    path_es_pub = node->advertise<nav_msgs::Path>("es_path", 5);
    path_gt_pub = node->advertise<nav_msgs::Path>("gt_path", 5);
    reprojected_es_pub = node->advertise<sensor_msgs::Image>("es_reprojected", 5);
    reprojected_gt_pub = node->advertise<sensor_msgs::Image>("gt_reprojected", 5);

    K << 718.539, 0., 620,
        0., 718.539, 188,
        0., 0., 1.;

    // 加载点云
    cloud_map = CloudMap::Ptr(new CloudMap());

    // 加载提取了Landmark过后的点云
    if (node->hasParam("cloud_map_path"))
    {
        string cloud_map_path;
        ROS_INFO("load cloud map");
        node->getParam("cloud_map_path", cloud_map_path);
        cloud_map->Load_PCD(cloud_map_path);
        ROS_INFO("%ld points loaded\n", cloud_map->point_count());
        PublishPointCloud(cloud_map->cloud, map_pub);
    }
    else
    {
        ROS_ERROR("No map loaded, quitting");
        return 0;
    }

    // 初始化粒子滤波器
    particle_cloud = pcl::PointCloud<Point>::Ptr(new pcl::PointCloud<Point>());
    init_particle_filter();
    pf.export_particles(particle_cloud);
    PublishPointCloud(particle_cloud, particle_cloud_pub);

    string pose_path, image_path, label_path;
    // 读取真实位姿
    if (node->hasParam("pose_path"))
    {
        node->getParam("pose_path", pose_path);
        poses_gt = load_pose(pose_path);
        ROS_INFO("%ld poses loaded", poses_gt.size());
    }
    else
    {
        ROS_ERROR("No poses loaded, quitting");
        return 0;
    }
    if (node->hasParam("image_path") && node->hasParam("label_path"))
    {
        node->getParam("image_path", image_path);
        node->getParam("label_path", label_path);
    }
    else
    {
        ROS_ERROR("image path undefined");
        return 0;
    }

    // 加载图像并处理
    boost::format image_fmt(image_path + "/%06d.png");
    boost::format label_fmt(label_path + "/%06d.png");
    PoseVector poses_es;
    PoseVector poses_gt_output;
    int i = 1;
    while (true)
    {
        std::string image_filename = (image_fmt % (i)).str();
        std::string label_filename = (label_fmt % (i)).str();
        if (!file_exists(image_filename) || !file_exists(label_filename))
            break;
        ROS_WARN("processing image %d", i);
        Image img = cv::imread(image_filename);
        Image label = cv::imread(label_filename, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
        ROS_INFO("height %d width %d", label.rows, label.cols);
        ROS_INFO("%s", image_filename.c_str());
        ROS_INFO("%s", label_filename.c_str());
        pf.processImage(img, label, poses_gt[i] - poses_gt[i - 1]);
        Pose rel_pose = poses_gt[i] - poses_gt[i - 1];
        ROS_INFO("rel_pose  %.4f %.4f %.4f %.4f", rel_pose(0), rel_pose(1), rel_pose(2), rel_pose(3));
        Pose pose_es = pf.get_pose();
        poses_es.push_back(pose_es);

        vector<Image> images_estimated = cloud_map->project(K, pose_es, 376, 1241);
        Image img_es = images_estimated[0];
        Image label_es = images_estimated[1];
        Pose pose_gt = poses_gt[i];
        poses_gt_output.push_back(pose_gt);

        vector<Image> images_gt = cloud_map->project(K, pose_gt, 376, 1241);
        Image img_gt = images_gt[0];
        Image label_gt = images_gt[1];

        Pose pose_err = pose_es - pose_gt;
        ROS_INFO("pose_es  %.4f %.4f %.4f %.4f", pose_es(0), pose_es(1), pose_es(2), pose_es(3));
        ROS_INFO("pose_gt  %.4f %.4f %.4f %.4f", pose_gt(0), pose_gt(1), pose_gt(2), pose_gt(3));
        ROS_INFO("pose_err %.4f %.4f %.4f %.4f", pose_err(0), pose_err(1), pose_err(2), pose_err(3));
        ROS_INFO("error %.4f", pose_err.norm());
        ROS_INFO("fitness_es %.4f", cloud_map->compute_fitness(K, img, label, pf.get_pose()));
        ROS_INFO("fitness_gt %.4f", cloud_map->compute_fitness(K, img, label, poses_gt[i]));
        pf.export_particles(particle_cloud);
        Point gt_point;
        gt_point.x = poses_gt[i](0);
        gt_point.y = poses_gt[i](1);
        gt_point.z = poses_gt[i](2);
        gt_point.rgba = 0xffffffff;
        particle_cloud->push_back(gt_point);
        PublishPointCloud(cloud_map->cloud, map_pub);
        PublishPointCloud(particle_cloud, particle_cloud_pub);
        PublishPose(pose_es, pose_es_pub);
        PublishPose(pose_gt, pose_gt_pub);
        PublishPath(poses_es, path_es_pub);
        PublishPath(poses_gt_output, path_gt_pub);
        PublishImage(img_es, reprojected_es_pub);
        PublishImage(img_gt, reprojected_gt_pub);
        spinOnce();
        i++;
        if(i > 100)
        {
            break;
        }
    }
    string pose_es_filename = pose_path.substr(0, pose_path.find_last_of('/')) + "/pose_es.txt";
    cout << pose_es_filename << endl;
    write_pose(pose_es_filename, poses_es);

    return 0;
}