#include "extract2DBoundingBoxFromMaskImage.h"
#include "extractPointCloudInBoundingBox.h"
#include "generatePointCloud.h"
#include "ReadPointCloud.h"


// Test RANSAC
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
// Test RANSAC

// Test convert
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
//

void Extract_PointCloud_from_Bounding_Box() {
	std::string depth_path = "D:/DATA/Research/demoData/depth/*png";
	std::string mask_path = "D:/DATA/Research/demoData/masks/*pbm";
	cutPointCloud(depth_path, mask_path);
}

void test_bounding_box_mask_image() {
	std::vector<std::string> mask_fileNames, depth_fileNames;
	std::string mask_path = "D:/DATA/Research/demoData/masks/*.pbm";
	std::string depth_path = "D:/DATA/Research/demoData/depth/*.png";

	cv::glob(mask_path, mask_fileNames, false);
	cv::glob(depth_path, depth_fileNames, false);

	std::size_t i = 0;
	for (auto const& f : mask_fileNames) {

		cv::Mat croppedImg;
		cv::Mat mask_img = cv::imread(mask_fileNames[i]);
		cv::Mat depth_img = cv::imread(depth_fileNames[i], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
		cv::resize(mask_img, mask_img, cv::Size(depth_img.cols, depth_img.rows), cv::INTER_LINEAR);

		cv::Mat mSource_Gray, mThreshold;
		cv::cvtColor(mask_img, mSource_Gray, cv::COLOR_BGR2GRAY);
		cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);
		cv::Mat Points;
		findNonZero(mThreshold, Points);
		cv::Rect Min_Rect = boundingRect(Points);

		int xMin = Min_Rect.tl().x;
		int xMax = Min_Rect.br().x;
		int yMin = Min_Rect.tl().y;
		int yMax = Min_Rect.br().y;

		cv::rectangle(depth_img, cv::Point(xMin + 10, yMin - 10), cv::Point(xMax + 20, yMax + 20), cv::Scalar(0, 0, 255));
		depth_img(cv::Rect(xMin + 10, yMin - 10, xMax + 20 - xMin - 10, yMax + 20 - yMin + 10)).copyTo(croppedImg);
		cv::imwrite("D:/DATA/Research/demoData/depth_crop/" + std::to_string(i) + ".png", croppedImg);
		//cv::imwrite("D:/DATA/Research/demoData/masks_bb/" + std::to_string(i) + ".png", depth_img);
		std::cout << i << std::endl;
		i++;
	}

}

void test_ransac() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

	cv::Mat depth = cv::imread("D:/DATA/Research/demoData/0.png", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
	const double intrinsic_n1[4] = { 571.15928878, 571.16352329, 311.07448762, 233.73421022 };


	const double fx = intrinsic_n1[0];
	const double fy = intrinsic_n1[1];
	const double cx = intrinsic_n1[2];
	const double cy = intrinsic_n1[3];

	for (int x = 0; x < depth.rows; x++) {
		for (int y = 0; y < depth.cols; y++) {
			pcl::PointXYZ p;

			double depth_val = depth.ptr<ushort>(x)[y];
			if (depth_val == 0) { continue; }

			p.z = depth_val;
			p.x = (y - cx) * p.z / fx;
			p.y = (x - cy) * p.z / fy;
			cloud->points.push_back(p);
		}
	}


	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(ransacFilter(cloud));
	while (!viewer.wasStopped())
	{
	}


}

void test_scaling_rgb2depth() {
	cv::Mat depth_img = cv::imread("D:/DATA/Research/demoData/test_new_method/NP1_0.png");
	cv::Mat rgb_img = cv::imread("D:/DATA/Research/demoData/test_new_method/NP1_0.jpg");
	cv::Mat mask_img = cv::imread("D:/DATA/Research/demoData/test_new_method/NP1_0_mask.pbm");

	// Find bounding box
	int xMin, xMax, yMin, yMax;
	int xMinD, xMaxD, yMinD, yMaxD;

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

	std::cout << xMin << " " << xMax << " " << yMin << " " << yMax << std::endl;

	// cv::Rect maskBoxes = cv::Rect(xMin, yMin, xMax - xMin, yMax - yMin);
	// 
	/*int cWidth = rgb_img.size().width;
	int cHeight = rgb_img.size().height;

	int dWidth = depth_img.size().width;
	int dHeight = depth_img.size().height;
	Eigen::Vector4d rBox = Eigen::Vector4d((double)maskBoxes.x / (double)cWidth, (double)maskBoxes.y / (double)cHeight, 
		(double)maskBoxes.width / (double)cWidth, (double)maskBoxes.height / (double)cHeight);
	cv::Rect rBB = cv::Rect(rBox.x() * dWidth, rBox.y() * dHeight, rBox.z() * dWidth, rBox.w() * dHeight);*/


	xMinD = ((double)xMin - 320) / 61696 - 310;
	yMinD = ((double)yMin - 240) / 61440 - 210;
	xMaxD = ((double)xMax - 320) / 61696 - 310;
	yMaxD = ((double)yMax - 240) / 61440 - 210;
	cv::Rect depthBox = cv::Rect(xMin, yMin, xMax - xMin, yMax - yMin);
	std::cout << xMinD << " " << xMaxD << " " << yMinD << " " << yMaxD << std::endl;

	/*cv::rectangle(depth_img, depthBox, cv::Scalar(0, 0, 255), 2);
	cv::imshow("D", depth_img);
	cv::waitKey(0);*/
}

/*
void testScalingBoundingBoxFromRGBImageToDepthImage()
{
	// Initializing
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage:: Initializing.\n";
	std::string colorImageFilePath = "D:/Solutions/CamTrioSDK/TestCamTrioSDK/Debug/BoxFromColorToDepthImage/colorImage.jpg";
	std::string depthImageFilePath = "D:/Solutions/CamTrioSDK/TestCamTrioSDK/Debug/BoxFromColorToDepthImage/depthImage.png";
	std::string colorBoxFilePath = "D:/Solutions/CamTrioSDK/TestCamTrioSDK/Debug/BoxFromColorToDepthImage/colorBox.txt";
	std::string pointCloudFilePath = "D:/Solutions/CamTrioSDK/TestCamTrioSDK/Debug/BoxFromColorToDepthImage/pointCloud.pcd";

	// Reading data
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage::  Reading data.\n";
	std::cout << "\t Reading color image ...\n";
	cv::Mat colorImage = cv::imread(colorImageFilePath);
	cv::Size colorImageSize = colorImage.size();
	int cWidth = colorImageSize.width; int cHeight = colorImageSize.height;
	double colorImageRatio = (double)cWidth / (double)cHeight;
	std::cout << "\t\t Color Image size: " << colorImage.size() << std::endl;
	std::cout << "\t\t Color Image Ratio: " << colorImageRatio << std::endl;

	std::cout << "\t Reading depth image ...\n";
	cv::Mat depthImage = cv::imread(depthImageFilePath);
	cv::Size depthImageSize = depthImage.size();
	int dWidth = depthImageSize.width; int dHeight = depthImageSize.height;
	double depthImageRatio = (double)dWidth / (double)dHeight;
	std::cout << "\t\t Depth Image size: " << depthImage.size() << std::endl;
	std::cout << "\t\t depthImageRatio: " << depthImageRatio << std::endl;

	std::cout << "\t Reading color boxes ...\n";
	Eigen::MatrixXd colorBoxes = readMatrixXdFromCSVFile(colorBoxFilePath, ' '); std::cout << "\t\t Box size: " << colorBoxes.rows() << std::endl;
	int xMin = colorBoxes(0, 0); int xMax = colorBoxes(0, 1);
	int yMin = colorBoxes(0, 2); int yMax = colorBoxes(0, 3);
	cv::Rect colorBox = cv::Rect(xMin, yMin, xMax - xMin, yMax - yMin); // xMin, yMin, width, height

	std::cout << "\t Reading point cloud ...\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pointCloudFilePath, *pointCloud); std::cout << "\t\t The size of point cloud: " << pointCloud->points.size() << std::endl;

	// Testing drawing color box on the color image
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage::  Testing drawing color box on the color image.\n";
	cv::Mat renderedColorImage = colorImage.clone();
	cv::rectangle(renderedColorImage, colorBox, cv::Scalar(0, 0, 255), 2);
	cv::imshow("RenderedColorImage", renderedColorImage);
	cv::waitKey(0);

	// Convert absolute coordinate to relative coordinate
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage:: Convert absolute coordinate to relative coordinate.\n";
	Eigen::Vector4d rColorBox = Eigen::Vector4d((double)colorBox.x / (double)cWidth, (double)colorBox.y / (double)cHeight,
		(double)colorBox.width / (double)cWidth, (double)colorBox.height / (double)cHeight); // xMin, yMin, width, height
	std::cout << "\t The absolute color box: " << colorBox << std::endl;
	std::cout << "\t The relative color box: " << rColorBox.transpose() << std::endl;

	// Convert relative coordinate to the depth image coordinate
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage:: Convert relative coordinate to the depth image coordinate.\n";
	cv::Rect dColorBox = cv::Rect(rColorBox.x() * dWidth, rColorBox.y() * dHeight, rColorBox.z() * dWidth, rColorBox.w() * dHeight);

	// Test the bounding box
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage:: Test the bounding box.\n";
	cv::Mat renderedDepthImage = depthImage.clone();
	cv::rectangle(renderedDepthImage, dColorBox, cv::Scalar(0, 0, 255), 2);
	cv::imshow("RenderedDepthImage", renderedDepthImage);
	cv::waitKey(0);

	// Finalizing
	std::cout << "testScalingBoundingBoxFromRGBImageToDepthImage:: Finalizing.\n";
}
*/

int main(int argc, char* argv[]) {
	//Extract_PointCloud_from_Bounding_Box();
	// test_bounding_box_mask_image();
	//test_ransac();
	// read_pcd("D:/DATA/Research/demoData/pointCloud/NP1_0.ply");
	test_scaling_rgb2depth();
	return 0;
}


