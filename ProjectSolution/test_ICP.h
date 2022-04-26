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
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>


#include "extract2DBoundingBoxFromMaskImage.h"
#include "extractPointCloudInBoundingBox.h"
#include "generatePointCloud.h"

#ifndef _test_icp_method
#define _test_icp_method
void process();
#endif