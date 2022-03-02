#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <librealsense2/hpp/rs_export.hpp>

#include <igl/writePLY.h>
#include <pcl/io/ply_io.h>

#ifndef _generate_point_cloud
#define _generate_point_cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr generate_point_cloud(cv::Mat depth_img, const double rgb_intrinsic[4]);
#endif

