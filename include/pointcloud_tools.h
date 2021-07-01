#ifndef __POINTCLOUD_TOOLS_H__
#define __POINTCLOUD_TOOLS_H__
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include <iostream>
#include "boost/format.hpp"
#include "types.h"
#include <string>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "tools.h"

typedef pcl::PointXYZRGBL Point;
#define SAVE_DEBUG_FILES
#endif