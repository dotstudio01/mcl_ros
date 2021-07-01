#ifndef __PARTICLE_FILTER_H__
#define __PARTICLE_FILTER_H__
#include "types.h"
#include "tools.h"
#include <vector>
#include "cloud_map.h"
#include "stdlib.h"
#include "time.h"
#include "math.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "ros/ros.h"

inline Pose ConstructPose(double x, double y, double z, double theta)
{
    Pose pose;
    pose << x, y, z, theta;
    return pose;
}

inline double getX(Pose pose)
{
    return pose(0, 0);
}

inline double getY(Pose pose)
{
    return pose(1, 0);
}

inline double getZ(Pose pose)
{
    return pose(2, 0);
}

inline double getTheta(Pose pose)
{
    return pose(3, 0);
}

struct Particle
{
    Particle(Pose pose, double weight, Particle *parent = 0, int childs = 0);
    Pose pose;
    double weight;
    double weightSum;
    Particle *parent;
    int childs;
};

typedef std::vector<Particle *> ParticleVector;

class ParticleFilter
{
public:
    ParticleFilter();
    void init(int particle_count, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
    void set_param(double std_trans, double std_rot);
    void set_map(CloudMap::Ptr map);
    void set_camera_param(CameraIntrinsics K);
    void processImage(Image image, Image label, Pose rel_pose);
    Pose get_pose();
    PoseVector get_pose_sequence();
    void print_particles();
    void export_particles(pcl::PointCloud<Point>::Ptr cloud);

private:
    void deleteParticle(Particle *particle);
    Pose sample_pose(Pose current_pose, Pose rel_pose);
    double std_trans, std_rot;
    ParticleVector particles;
    CloudMap::Ptr map;
    CameraIntrinsics K;
};

#endif