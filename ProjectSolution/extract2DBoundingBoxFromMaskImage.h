#include <iostream>
#include <string>
#include <string.h>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Dense>

#ifndef _extract_bounding_box
#define _extract_bounding_box
void get_bounding_box_from_mask_image(cv::Mat mask_img, cv::Mat depth_img, double& bbX, double& bbY, double& bbWidth, double& bbHeight);
#endif // !_extract_bounding_box
