# Monte Carlo Localition Using Landmark Map
**This is the implementation of Monte Carlo localization used in landmark learning.

As described in the ICCV 2015 paper **PoseNet: A Convolutional Network for Real-Time 6-DOF Camera Relocalization** Alex Kendall, Matthew Grimes and Roberto Cipolla [http://mi.eng.cam.ac.uk/projects/relocalisation/]

## Getting Started

 * Install ROS Kinetic
 * Install pcl-1.8 library
 * Prepare atttention maps extracted using LandmarkNet
Download the KITTI dataset [from here.](http://www.cvlibs.net/datasets/kitti/index.php)

## Usage
 * Build project using ```catkin_make```
 * Landmark map construction using ```rosrun mcl_ros pointcloud_tools build_map [data_folder]```
 * Perform Localization using ```roslaunch mcl_ros mcl.launch"

