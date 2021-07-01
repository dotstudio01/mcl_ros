#include "particle_filter.h"

Particle::Particle(Pose pose, double weight, Particle *parent, int childs)
{
    this->pose = pose;
    this->weight = weight;
    this->weightSum = weight;
    this->parent = parent;
    this->childs = childs;
}

ParticleFilter::ParticleFilter()
{
}

// 初始化粒子滤波器
void ParticleFilter::init(int particle_count, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
    double weight = 1.0 / (double)particle_count;
    Particle *particle;
    // Particle *particle = new Particle(ConstructPose(0, 0, 0,0), weight);
    // particles.push_back(particle);
    for (int i = 0; i < particle_count; i++)
    {
        double x = (xmax - xmin) * randDouble() + xmin;
        double y = (ymax - ymin) * randDouble() + ymin;
        double z = (zmax - zmin) * randDouble() + zmin;
        double theta = M_PI * 2 * randDouble() - 2 * M_PI;
        particle = new Particle(ConstructPose(x, y, z, theta), weight);
        particles.push_back(particle);
    }
}

void ParticleFilter::set_param(double std_trans, double std_rot)
{
    this->std_trans = std_trans;
    this->std_rot = std_rot;
}

void ParticleFilter::set_map(CloudMap::Ptr map)
{
    this->map = map;
}

void ParticleFilter::set_camera_param(CameraIntrinsics K)
{
    this->K = K;
}

void ParticleFilter::deleteParticle(Particle *particle)
{
    if (particle->parent)
    {
        particle->parent->childs--;
        if (particle->parent->childs == 0)
        {
            deleteParticle(particle->parent);
        }
    }
    delete particle;
}

Pose ParticleFilter::sample_pose(Pose pose, Pose rel_pose)
{
    double newX = getX(pose) + rel_pose[0]  + randGaussian(this->std_trans);
    double newY = getY(pose) + rel_pose[1]  + randGaussian(this->std_trans);
    double newZ = getZ(pose) + rel_pose[2]  + randGaussian(this->std_trans);
    double newTheta = getTheta(pose) + rel_pose[3]  + randGaussian(this->std_rot);
    while (abs(newTheta) > M_PI)
    {
        newTheta += newTheta > 0 ? -2 * M_PI : 2 * M_PI;
    }
    return ConstructPose(newX, newY, newZ, newTheta);
}

void ParticleFilter::print_particles()
{
    for (int i = 0; i < particles.size(); i++)
    {
        std::cout << "particle " << i << " " << particles[i]->pose.transpose() << " " << particles[i]->weight << " " << std::endl;
    }
}

void ParticleFilter::export_particles(pcl::PointCloud<Point>::Ptr cloud)
{
    cloud->clear();
    Point pt;
    pt.x = 0;
    pt.y = 0;
    pt.z = 0;
    pt.r = 255;
    pt.g = 255;
    pt.b = 0;
    pt.a = 255;
    cloud->push_back(pt);
    pt.x = 100;
    pt.y = 0;
    pt.z = 0;
    pt.r = 255;
    pt.g = 0;
    pt.b = 0;
    pt.a = 255;
    cloud->push_back(pt);
    pt.x = 0;
    pt.y = 100;
    pt.r = 0;
    pt.g = 255;
    cloud->push_back(pt);
    pt.y = 0;
    pt.z = 100;
    pt.g = 0;
    pt.b = 255;
    cloud->push_back(pt);
    double max_weight = 0;
    for (int i = 0; i < particles.size(); i++)
    {
        max_weight = max_weight > particles[i]->weight ? max_weight : particles[i]->weight;
    }

    for (int i = 0; i < particles.size(); i++)
    {
        int w = (int)(particles[i]->weight / max_weight * 255.0);
        pt.x = particles[i]->pose(0);
        pt.y = particles[i]->pose(1);
        pt.z = particles[i]->pose(2);
        pt.r = colormap_r[w];
        pt.g = colormap_g[w];
        pt.b = colormap_b[w];
        pt.a = 255;
        cloud->push_back(pt);
    }
}

void ParticleFilter::processImage(Image image, Image label, Pose rel_pose)
{
    ParticleVector prevParticles = particles;
    particles = ParticleVector();
    // 根据运动模型生成新的particle
    Particle *best_particle = NULL;
    for (ParticleVector::iterator it = prevParticles.begin(); it != prevParticles.end(); it++)
    {
        Particle *prevParticle = *it;
        Pose newPose = sample_pose(prevParticle->pose, rel_pose);
        double newWeight = this->map->compute_fitness(this->K, image, label, newPose);

        Particle *particle = new Particle(newPose, newWeight, prevParticle, 0);
        prevParticle->childs++;
        particle->weightSum = prevParticle->weightSum + newWeight;
        if (best_particle == NULL || particle->weight > best_particle->weight)
        {
            best_particle = particle;
        }
        particles.push_back(particle);
    }

    // // 尝试优化best_particle的位姿
    // Pose pose_offset[8];
    // pose_offset[0] << 0.1, 0, 0, 0;
    // pose_offset[1] << -0.1, 0, 0, 0;
    // pose_offset[2] << 0, 0.1, 0, 0;
    // pose_offset[3] << 0, -0.1, 0, 0;
    // pose_offset[4] << 0, 0, 0.1, 0;
    // pose_offset[5] << 0, 0, -0.1, 0;
    // pose_offset[6] << 0, 0, 0, 1 * M_PI / 180.0;
    // pose_offset[7] << 0, 0, 0, -1 * M_PI / 180.0;
    // ROS_INFO("optimizing pose");
    // ROS_INFO("  before: %.4f %.4f %.4f %.4f", best_particle->pose(0), best_particle->pose(1), best_particle->pose(2), best_particle->pose(3));
    // ROS_INFO("  score:  %.4f", best_particle->weight);

    // while (true)
    // {
    //     double max_score = 0;
    //     int max_idx;
    //     for (int i = 0; i < 8; i++)
    //     {
    //         Pose new_pose = best_particle->pose + pose_offset[i];
    //         double new_score = map->compute_fitness(K, image, new_pose);
    //         if (new_score > max_score)
    //         {
    //             max_score = new_score;
    //             max_idx = i;
    //         }
    //     }
    //     if (max_score > best_particle->weight)
    //     {
    //         best_particle->pose = best_particle->pose + pose_offset[max_idx];
    //         best_particle->weight = max_score;
    //         best_particle->weightSum = best_particle->parent->weightSum + max_score;
    //     }
    //     else
    //     {
    //         break;
    //     }
    // }
    // // ROS_INFO("optimizing pose");
    // ROS_INFO("  after: %.4f %.4f %.4f %.4f", best_particle->pose(0), best_particle->pose(1), best_particle->pose(2), best_particle->pose(3));
    // ROS_INFO("  score:  %.4f", best_particle->weight);
    double normFactor = 0;
    for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
    {
        normFactor += (*it)->weight;
    }
    // 归一化，并计算
    // GMapping中采用的是Log归一化
    double neff;
    for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
    {
        Particle *particle = *it;
        particle->weight /= normFactor;
        particle->weightSum = particle->parent->weightSum + particle->weight;
        neff += particle->weight * particle->weight;
    }
    neff = 1.0 / neff;
    std::cout << "neff:" << neff << std::endl;
    // 判断是否需要resample
    if (neff < 0.8 * particles.size())
    {
        std::cout << "resampling" << std::endl;
        std::vector<int> indexes(particles.size());
        double cweight = 0;
        double interval = 1.0 / (double)particles.size();
        double target = interval * randDouble();
        ParticleVector resampledParticles;
        for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
        {
            bool used = false;
            cweight += (*it)->weight;
            while (cweight > target)
            {
                target += interval;
                Particle *resampledParticle = new Particle(**it);
                resampledParticle->pose(0) += randGaussian(this->std_trans);
                resampledParticle->pose(1) += randGaussian(this->std_trans);
                resampledParticle->pose(2) += randGaussian(this->std_trans);
                resampledParticle->pose(3) += randGaussian(this->std_rot);
                resampledParticles.push_back(resampledParticle);
                if (resampledParticle->parent)
                {
                    resampledParticle->parent->childs++;
                }
            }
            deleteParticle(*it);
        }
        particles = resampledParticles;
        // 重新归一化
        // 计算normFactor
        normFactor = 0;
        for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
            normFactor += (*it)->weight;
        // 归一化 ，重新计算weighSum
        for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
        {
            Particle *particle = *it;
            particle->weight /= normFactor;
            particle->weightSum = particle->parent->weightSum + particle->weight;
            neff += particle->weight * particle->weight;
        }
    }
}

Pose ParticleFilter::get_pose()
{
    Particle *best_particle = particles[0];
    for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
    {
        if ((*it)->weightSum > best_particle->weightSum)
        {
            best_particle = *it;
        }
    }
    return best_particle->pose;
}

PoseVector ParticleFilter::get_pose_sequence()
{
    PoseVector poses;
    Particle *best_particle = particles[0];
    for (ParticleVector::iterator it = particles.begin(); it != particles.end(); it++)
    {
        if ((*it)->weightSum > best_particle->weightSum)
        {
            best_particle = *it;
        }
    }
    while (best_particle)
    {
        poses.push_back(best_particle->pose);
        best_particle = best_particle->parent;
    }
    return poses;
}