#include "extract2DBoundingBoxFromMaskImage.h"

void get_bounding_box_from_mask_image(cv::Mat mask_img, int& xMin, int& yMin, int& xMax, int& yMax) {

	cv::Mat mSource_Gray, mThreshold;
	cv::cvtColor(mask_img, mSource_Gray, cv::COLOR_BGR2GRAY);
	cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);
	cv::Mat Points;
	findNonZero(mThreshold, Points);
	cv::Rect Min_Rect = boundingRect(Points);

	xMin = Min_Rect.tl().x;
	xMax = Min_Rect.br().x;
	yMin = Min_Rect.tl().y;
	yMax = Min_Rect.br().y;	
}