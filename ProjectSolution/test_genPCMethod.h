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
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include "extract2DBoundingBoxFromMaskImage.h"
#include "extractPointCloudInBoundingBox.h"
#include "generatePointCloud.h"

#ifndef _read_csv
#define _read_csv
Eigen::MatrixXf readCSV(std::string file, int rows, int cols);
#endif

#ifndef _check_natural_sort
#define _check_natural_sort
bool compareNat(const std::string& a, const std::string& b);
#endif

#ifndef _define_process_each_view
#define _define_process_each_view
void choose_matrix_each_view(std::string data_path);
#endif

#ifndef _test_process_one_frame
#define _test_process_one_frame
void test_process_frame(std::string data_path);
#endif