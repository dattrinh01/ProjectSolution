#include "extract2DBoundingBoxFromMaskImage.h"

void get_bounding_box_from_mask_image(cv::Mat mask_img, cv::Mat depth_img, double&bbX, double&bbY, double&bbWidth, double&bbHeight) {

	cv::Mat mSource_Gray, mThreshold;
	cv::cvtColor(mask_img, mSource_Gray, cv::COLOR_BGR2GRAY);
	cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);
	cv::Mat Points;
	findNonZero(mThreshold, Points);
	cv::Rect Min_Rect = boundingRect(Points);

	cv::Rect maskBoxes = cv::Rect(Min_Rect.tl().x, Min_Rect.tl().y, Min_Rect.br().x - Min_Rect.tl().x, Min_Rect.br().y - Min_Rect.tl().y);

	int cWidth = mask_img.size().width;
	int cHeight = mask_img.size().height;

	int dWidth = depth_img.size().width;
	int dHeight = depth_img.size().height;
	Eigen::Vector4d rBox = Eigen::Vector4d((double)maskBoxes.x / (double)cWidth, (double)maskBoxes.y / (double)cHeight,
		(double)maskBoxes.width / (double)cWidth, (double)maskBoxes.height / (double)cHeight);
	// cv::Rect rBB = cv::Rect(rBox.x() * dWidth + 10, rBox.y() * dHeight, rBox.z() * dWidth + 10, rBox.w() * dHeight + 10);
	bbX = rBox.x() * dWidth + 10;
	bbY = rBox.y() * dHeight;
	bbWidth = rBox.z() * dWidth + 10;
	bbHeight = rBox.w() * dHeight + 10;

}