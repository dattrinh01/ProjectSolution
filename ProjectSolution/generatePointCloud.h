#include <iostream>
#include <math.h>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


#ifndef _generate_point_cloud
#define _generate_point_cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(cv::Mat depth_img, const double depth_intrinsic[4]);
#endif