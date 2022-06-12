#ifndef _TEST_SPLIT_IMAGE
#define _TEST_SPLIT_IMAGE

#include <iostream>
#include <vector>
#include <string.h>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr test_generatePointCloudFromDepthImage(cv::Mat depth_img, const double depth_intrinsic[4]);
void splitFromMergeImage();

#endif // _TEST_SPLIT_IMAGE
