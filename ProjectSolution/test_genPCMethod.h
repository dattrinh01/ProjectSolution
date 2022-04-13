#include <iostream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <sstream>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "extract2DBoundingBoxFromMaskImage.h"
#include "generatePointCloud.h"

#ifndef _save_ply_point_cloud
#define _save_ply_point_cloud
std::string createPLYFileNames(std::string path, std::string toErase);
#endif

#ifndef _test_gen_point_cloud_method
#define _test_gen_point_cloud_method
void test_generate_new_point_cloud_method(std::string depth_path, std::string mask_path);
#endif

#ifndef _read_csv
#define _read_csv
void read_csv_file(std::string csv_path);
#endif