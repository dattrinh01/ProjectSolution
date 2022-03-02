#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#ifndef _calib_function
#define _calib_function
void calib(std::vector<cv::String>fileNames, int patternSizeX, int patternSizeY, cv::Matx33f& cameraMatrix, cv::Vec<float, 5>& distortionMatrix);
#endif // !_calib_function
