#include <iostream>
#include <string>
#include <string.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#ifndef _cal_projection_matrix
#define _cal_projection_matrix
cv::Mat computeProjMat(cv::Mat camMat, cv::Mat rotVec, cv::Mat transVec);
#endif