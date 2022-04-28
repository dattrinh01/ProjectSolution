#include "extract2DBoundingBoxFromMaskImage.h"
#include "extractPointCloudInBoundingBox.h"
#include "generatePointCloud.h"
#include "ReadPointCloud.h"

#include "test_genPCMethod.h"
#include "test_ICP.h"
#include "test_math.h"

#include "transformToOriginalPoints.h"

// Test RANSAC
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
//
// Test transforms point cloud
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
//
void Extract_PointCloud_from_Bounding_Box();
void test_bounding_box_mask_image();
void test_ransac();
void test_scaling_rgb2depth();
void testScalingBoundingBoxFromRGBImageToDepthImage();
void test_transform_point_cloud();
void drawAxes(cv::Mat& src_, cv::Mat& dst_, std::vector<cv::Point2d>& imgPts_, std::vector<cv::Point2f>& cornersSP_);
void test_pose_estimation();

int main(int argc, char* argv[]) {
	applyConvertPointCloudToMatrix("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP1_pc.ply");
	return 0;
}


void Extract_PointCloud_from_Bounding_Box() {
	std::string depth_path = "D:/DATA/Research/demoData/red_bull/depth/*.png";
	std::string mask_path = "D:/DATA/Research/demoData/red_bull/masks/*.pbm";
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


	/*pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(ransacFilter(cloud));
	while (!viewer.wasStopped())
	{
	}*/


}

void test_scaling_rgb2depth() {
	/*cv::Mat depth_img = cv::imread("D:/DATA/Research/demoData/quaker_chewy_dipps_peanut_butter_chocolate/depth/NP1_0.png");
	cv::Mat rgb_img = cv::imread("D:/DATA/Research/demoData/quaker_chewy_dipps_peanut_butter_chocolate/rgb/NP1_0.jpg");
	cv::Mat mask_img = cv::imread("D:/DATA/Research/demoData/quaker_chewy_dipps_peanut_butter_chocolate/masks/NP1_0_mask.pbm");*/

	std::vector<std::string> depth_fileNames, mask_fileNames;
	std::string depth_path = "D:/DATA/Research/demoData/windex/depth/*.png";
	std::string mask_path = "D:/DATA/Research/demoData/windex/masks/*.pbm";
	cv::glob(depth_path, depth_fileNames, false);
	cv::glob(mask_path, mask_fileNames, false);

	std::size_t i = 0;

	for (auto const& f : depth_fileNames) {
		cv::Mat depth_img = cv::imread(depth_fileNames[i]);
		cv::Mat mask_img = cv::imread(mask_fileNames[i]);
		int xMin, xMax, yMin, yMax;

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
		cv::Rect maskBoxes = cv::Rect(xMin, yMin, xMax - xMin, yMax - yMin);
		/*cv::rectangle(rgb_img, maskBoxes, cv::Scalar(0, 0, 255), 2);
		cv::imshow("1", rgb_img);*/

		int cWidth = mask_img.size().width;
		int cHeight = mask_img.size().height;

		int dWidth = depth_img.size().width;
		int dHeight = depth_img.size().height;
		Eigen::Vector4d rBox = Eigen::Vector4d((double)maskBoxes.x / (double)cWidth, (double)maskBoxes.y / (double)cHeight,
			(double)maskBoxes.width / (double)cWidth, (double)maskBoxes.height / (double)cHeight);
		cv::Rect rBB = cv::Rect(rBox.x() * dWidth + 10, rBox.y() * dHeight, rBox.z() * dWidth + 10, rBox.w() * dHeight + 10);

		cv::rectangle(depth_img, rBB, cv::Scalar(0, 0, 255), 2);
		cv::imwrite("D:/DATA/Research/demoData/windex/depth_bb/" + std::to_string(i) + ".png", depth_img);
		std::cout << "D:/DATA/Research/demoData/windex/depth_bb/" + std::to_string(i) + ".png" << std::endl;
		i++;
	}
	
}



void test_transform_point_cloud() {

	std::string pathPointCloud = "D:/DATA/Research/demoData/red_bull/NP1_0.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPLYFile(pathPointCloud, *cloud);
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	float theta = 90.0f * (M_PI / 180.0f);
	transformation(0, 0) = cos(theta);
	transformation(0, 1) = -sin(theta);
	transformation(1, 0) = sin(theta);
	transformation(1, 1) = cos(theta);

	transformation(0, 3) = 1.0f;

	pcl::transformPointCloud(*cloud, *cloudTransformed, transformation);
	pcl::visualization::CloudViewer viewer("cloud");
	viewer.showCloud(cloudTransformed);
	while (!viewer.wasStopped()) {

	}
	pcl::io::savePLYFileBinary("D:/DATA/Research/demoData/red_bull/raw_pc.ply", *cloudTransformed);

}

void test_camera_calib(std::vector<std::vector<cv::Point3f> >&objpoints, std::vector<std::vector<cv::Point2f> >&imgpoints)
{
	std::vector<cv::Point3f> objp;
	int CHECKERBOARD[2]{ 9,8 };

	for (int i{ 0 }; i < CHECKERBOARD[1]; i++)
	{
		for (int j{ 0 }; j < CHECKERBOARD[0]; j++)
			objp.push_back(cv::Point3f(j, i, 0));
	}

	std::vector<cv::String> images;
	std::string path = "D:/DATA/Research/DrNhu/demoData/chessboard/*.jpg";
	cv::glob(path, images);
	cv::Mat frame, gray;
	std::vector<cv::Point2f> corner_pts;

	for (int i{ 0 }; i < images.size(); i++)
	{
		frame = cv::imread(images[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		std::cout << i << std::endl;

		if (gray.cols > 1200) {
			cv::Mat gray_temp;
			cv::GaussianBlur(gray, gray_temp, cv::Size(0,0), 105);
			cv::addWeighted(gray, 1.8, gray_temp, -0.8, 0, gray);
		}

		int flag_find_chessboard = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FILTER_QUADS;

		bool success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, flag_find_chessboard);
		if (success == 1)
		{
			cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
			cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			objpoints.push_back(objp);
			imgpoints.push_back(corner_pts);

			//std::cout << objp << std::endl;

			//cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

		}
		/*cv::imshow("Find chessboard", frame);
		cv::waitKey(1);*/
	}

	cv::Mat R, T;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

	std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;
	std::cout << "Rotation vector : " << R << std::endl;
	std::cout << "Translation vector : " << T << std::endl;
	std::cout << "--------------" << std::endl;

	
}

void drawAxes(cv::Mat& src_, cv::Mat& dst_,
	std::vector<cv::Point2d>& imgPts_,
	std::vector<cv::Point2f>& cornersSP_) {
	src_.copyTo(dst_);
	cv::arrowedLine(dst_, cornersSP_[0], imgPts_[0], cv::Scalar(0, 0, 255), 2,
		cv::LINE_AA, 0);
	cv::arrowedLine(dst_, cornersSP_[0], imgPts_[1], cv::Scalar(0, 255, 0), 2,
		cv::LINE_AA, 0);
	cv::arrowedLine(dst_, cornersSP_[0], imgPts_[2], cv::Scalar(255, 0, 0), 2,
		cv::LINE_AA, 0);
}

void test_pose_estimation() {

	cv::Size patternSize_cvS(12, 8);
	cv::Mat cameraMatrix_Mat = (cv::Mat_<float>(3, 3) << 
		1.0272067707356030e+03, 0., 6.3950000000000000e+02, 0.,
		1.0272067707356030e+03, 3.5950000000000000e+02, 0., 0.,1.);
	std::vector<double> distCoeffs_vecDbt = { -4.1975909041818131e-02, 6.1113602868631364e-02, 0., 0., -1.8944263773968928e-01 };

	cv::Mat frame_Mat, outputFrame_Mat, rvec_Mat, tvec_Mat, gray_Mat;
	std::vector<cv::Point2f> corners_vecP2f;
	std::vector<cv::Point2d> imgPts_vecP2d;
	std::vector<cv::Point3d> boardPts_vecP3d;
	std::vector<cv::Point3d> axis_vecP3d;

	for (int i{ 0 }; i < patternSize_cvS.height; i++)
	{
		for (int j{ 0 }; j < patternSize_cvS.width; j++)
		{
			boardPts_vecP3d.push_back(
				cv::Point3f(i, j, 0)
			);
		}
	}

	axis_vecP3d.push_back(
		cv::Point3d(3.0, 0.0, 0.0)
	);

	axis_vecP3d.push_back(
		cv::Point3d(0.0, 3.0, 0.0)
	);

	axis_vecP3d.push_back(
		cv::Point3d(0.0, 0.0, -3.0)
	);

	cv::Mat rotMat;

	std::vector<cv::String> images;
	std::string path = "D:/DATA/Research/DrNhu/demoData/chessboard/*.jpg";
	cv::glob(path, images);

	for (unsigned int i{ 0 }; i < images.size(); i++)
	{
		frame_Mat = cv::imread(images[i]);
		
		cv::cvtColor(frame_Mat, gray_Mat, cv::COLOR_BGR2GRAY);
		cv::findChessboardCorners(gray_Mat, patternSize_cvS, corners_vecP2f, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
		cv::TermCriteria termCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
		cv::cornerSubPix(gray_Mat, corners_vecP2f, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
		cv::solvePnPRansac(boardPts_vecP3d, corners_vecP2f, cameraMatrix_Mat, distCoeffs_vecDbt, rvec_Mat, tvec_Mat, false, 100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_EPNP);

		cv::projectPoints(axis_vecP3d, rvec_Mat, tvec_Mat, cameraMatrix_Mat, distCoeffs_vecDbt, imgPts_vecP2d, cv::noArray(), 0.0);
		std::cout << std::string(images[i]) << std::endl;
		drawAxes(frame_Mat, outputFrame_Mat, imgPts_vecP2d, corners_vecP2f);
		cv::Rodrigues(rvec_Mat, rotMat);

		/*cv::imshow("poseOP", outputFrame_Mat);
		cv::waitKey(0);*/



		/*cv::Mat rotMat(3, 3, CV_64F), rotTransMat(3, 4, CV_64F);
		cv::Rodrigues(rvec_Mat, rotMat);
		cv::hconcat(rotMat, tvec_Mat, rotTransMat);
		std::cout << cameraMatrix_Mat * rotTransMat;*/
	}
}


