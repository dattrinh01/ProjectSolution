#include <iostream>
#include <string>
#include <string.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>


#ifndef _read_point_cloud_bigbird
#define _read_point_cloud_bigbird
void read_pcd(std::string pointCloudPath);
#endif