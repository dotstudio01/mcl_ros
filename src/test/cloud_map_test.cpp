#include "cloud_map.h"
#include "tools.h"
#include "boost/format.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
int main(int argc, char **argv)
{
    CloudMap::Ptr cloud_map = CloudMap::Ptr(new CloudMap());
    cout << "loading cloudmap" << endl;
    // cloud_map->Load_PLY("data/merged.ply");
    cloud_map->Load_PCD("data/test/map.pcd");
    cout << cloud_map->point_count() << " points loaded" << endl;
    // cloud_map->apply_filter(0.8);
    
    CameraIntrinsics K;
    K << 718.539, 0., 620,
        0., 718.539, 187.5,
        0., 0., 1.;
    boost::format fmt("data/test/reprojected/%06d.png");
    PoseVector poses = load_pose("data/test/pose.txt");
    for(int i = 0; i < 100; i++)
    {
        vector<Image> images = cloud_map->project(K, poses[i], 376, 1241);
        Image img = images[0];
        cv::imwrite((fmt % (i)).str(), img);
    }
    return 0;
}