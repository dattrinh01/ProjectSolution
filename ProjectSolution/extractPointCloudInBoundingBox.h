#include <iostream>
#include <math.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "extract2DBoundingBoxFromMaskImage.h"
#include "generatePointCloud.h"

#ifndef _generate_file_names
#define _generate_file_names
std::string createPLYFileNames(std::string path, std::string toErase);
#endif

#ifndef _cut_out_pc
#define _cut_out_pc
void cutPointCloud(std::string depth_path, std::string mask_path);
#endif

