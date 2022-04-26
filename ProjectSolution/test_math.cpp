#include "test_math.h"

cv::Mat computeProjMat(cv::Mat camMat, cv::Mat rotVec, cv::Mat transVec)
{

	cv::Mat rotMat;
	cv::Mat I = cv::Mat::eye(cv::Size(3, 3), CV_32F);
	cv::Rodrigues(rotVec, rotMat);
	cv::Mat minus_tvecs = -1 * transVec;

	std::vector<cv::Mat> IC_vec = { rotMat, minus_tvecs };
	cv::Mat IC_Mat;
	cv::hconcat(IC_vec, IC_Mat);
	std::cout << rotMat << std::endl; 
}