# Monte Carlo Localition Using Landmark Map
** This is the implementation of Monte Carlo localization used in landmark learning.

## Getting Started

 * Install ROS Kinetic
 * Install pcl-1.8 library
 * Prepare atttention maps extracted using LandmarkNet
Download the KITTI dataset [from here.](http://www.cvlibs.net/datasets/kitti/index.php)

## Usage
 * Build project using ```catkin_make```
 * Landmark map construction using ```rosrun mcl_ros pointcloud_tools build_map [data_folder]```
 * Perform Localization using ```roslaunch mcl_ros mcl.launch"

