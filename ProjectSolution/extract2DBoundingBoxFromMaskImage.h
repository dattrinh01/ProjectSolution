#include <iostream>
#include <string>
#include <string.h>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#ifndef _extract_bounding_box
#define _extract_bounding_box
void get_bounding_box_from_mask_image(cv::Mat mask_img, int& xMin, int& yMin, int& xMax, int& yMax);
#endif // !_extract_bounding_box
