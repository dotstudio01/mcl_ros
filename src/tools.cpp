#include "tools.h"

double randDouble()
{
    return (double)rand() / (double)RAND_MAX;
}

float randFloat()
{
    return (float)rand() / (float)RAND_MAX;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double randGaussian(double sigma)
{
    double x1, x2, w;
    double r;
    do
    {
        do
        {
            r = randDouble();
        } while (r == 0.0);
        x1 = 2.0 * r - 1.0;
        do
        {
            r = randDouble();
        } while (r == 0.0);
        x2 = 2.0 * randDouble() - 1.0;
        w = x1 * x1 + x2 * x2;
    } while (w > 1.0 || w == 0.0);

    return (sigma * x2 * sqrt(-2.0 * log(w) / w));
}

PoseVector load_pose(const std::string &file_name)
{
    PoseVector poses;
    std::ifstream fs(file_name);
    while (!fs.eof())
    {
        Pose pose;
        Eigen::Matrix<double, 1, 7> pose_quat;
        double idx;
        fs >> idx;
        for (int j = 1; j < 8; j++)
        {
            fs >> pose_quat(0, j - 1);
        }
        pose << pose_quat(0, 0), pose_quat(0, 1), pose_quat(0, 2), atan2(-pose_quat(6), pose_quat(3)) * 2;
        poses.push_back(pose);
    }
    return poses;
}

void write_pose(const std::string &file_name, PoseVector &poses)
{
    std::fstream fs(file_name, std::ios::out);
    for (int i = 0; i < poses.size(); i++)
    {
        fs << i << " ";
        for(int j = 0; j < 3; j++)
        {
            fs << poses[i](j) << " ";
        }
        fs << 0 << " " << -sin(2 * poses[i](3)) << " "<< 0 << " " << cos(2 * poses[i](3)) << std::endl;
    }
}

bool file_exists(const std::string &name)
{
    if (FILE *file = fopen(name.c_str(), "r"))
    {
        fclose(file);
        return true;
    }
    else
    {
        return false;
    }
}