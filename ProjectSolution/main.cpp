#include "extract2DBoundingBoxFromMaskImage.h"
#include "extractPointCloudInBoundingBox.h"
#include "generatePointCloud.h"


// Test RANSAC
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
// Test RANSAC

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
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}


}

int main(int argc, char* argv[]) {
	Extract_PointCloud_from_Bounding_Box();
	//test_bounding_box_mask_image();
	//test_ransac();


	return 0;
}