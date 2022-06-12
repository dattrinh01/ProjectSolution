#include "splitImage.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr test_generatePointCloudFromDepthImage(cv::Mat depth_img, const double depth_intrinsic[4])
{
	const double fx = depth_intrinsic[0];
	const double fy = depth_intrinsic[1];
	const double cx = depth_intrinsic[2];
	const double cy = depth_intrinsic[3];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int x = 0; x < depth_img.rows; x++) {
		for (int y = 0; y < depth_img.cols; y++) {
			pcl::PointXYZ p;

			double depth_val = depth_img.ptr<ushort>(x)[y];
			if (depth_val == 0) { continue; }
			p.z = depth_val;
			p.x = (y - cx) * p.z / fx;
			p.y = (x - cy) * p.z / fy;
			cloud->points.push_back(p);

		}
	}
	return cloud;
}

void splitFromMergeImage()
{
	cv::Mat img_h = cv::imread("D:/DATA/Research/DrNhu/ProjectSolutionFolder/datasetGeneration/Outputs/Object12/depth/NP1_0_obj1_merge_NP1_0_obj2_v.png", cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);

	unsigned short img_width = img_h.size().width;
	unsigned short img_height = img_h.size().height;

	std::cout << img_h.size().width << std::endl;
	std::cout << img_h.size().height << std::endl;

	const double intrinsic_n1_1[4] = { 570.31691719, 570.31988743, 314.9278689, 228.59060364 };

	cv::Mat raw_img_1 = cv::imread("D:/DATA/Research/DrNhu/ProjectSolutionFolder/datasetGeneration/Inputs/object_1/depth/NP1_0.png", cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
	cv::Mat raw_img_2 = cv::imread("D:/DATA/Research/DrNhu/ProjectSolutionFolder/datasetGeneration/Inputs/object_2/depth/NP1_0.png", cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);

	double boundingArr[2][5] = { 0 ,0.497656, 0.305908, 0.071875, 0.0981445, 1, 0.498828, 0.794678, 0.0960938, 0.120605 };

	boundingArr[0][1] *= 1280;
	boundingArr[0][2] *= 2048;
	boundingArr[0][3] *= 1280;
	boundingArr[0][4] *= 2048;

	boundingArr[0][1] = boundingArr[0][1] - boundingArr[0][3] / 2;
	boundingArr[0][2] = boundingArr[0][2] - boundingArr[0][4] / 2;
	boundingArr[0][3] = boundingArr[0][1] + boundingArr[0][3];
	boundingArr[0][4] = boundingArr[0][2] + boundingArr[0][4];

	boundingArr[0][1] = (boundingArr[0][1] / 1280 * 640) + 10;
	boundingArr[0][2] = (boundingArr[0][2] / 2048 * 960);
	boundingArr[0][3] = (boundingArr[0][3] / 1280 * 640) + 20;
	boundingArr[0][4] = (boundingArr[0][4] / 2048 * 960) + 20;

	boundingArr[1][1] *= 1280;
	boundingArr[1][2] *= 2048;
	boundingArr[1][3] *= 1280;
	boundingArr[1][4] *= 2048;

	boundingArr[1][1] = boundingArr[1][1] - boundingArr[1][3] / 2;
	boundingArr[1][2] = boundingArr[1][2] - boundingArr[1][4] / 2;
	boundingArr[1][3] = boundingArr[1][1] + boundingArr[1][3];
	boundingArr[1][4] = boundingArr[1][2] + boundingArr[1][4];

	boundingArr[1][1] = (boundingArr[1][1] / 1280 * 640) + 10;
	boundingArr[1][2] = (boundingArr[1][2] / 2048 * 960);
	boundingArr[1][3] = (boundingArr[1][3] / 1280 * 640) + 20;
	boundingArr[1][4] = (boundingArr[1][4] / 2048 * 960) + 20;

	boundingArr[1][2] -= 480;
	boundingArr[1][4] -= 480;

	for (unsigned int i = 0; i < 2; i++)
	{
		for (unsigned int j = 0; j < 5; j++)
		{
			std::cout << boundingArr[i][j] << " ";
		}
		std::cout << std::endl;
	}

	/*if (img_width > img_height)
	{
		cv::Rect rect_1(boundingArr[0][1], boundingArr[0][2], boundingArr[0][3] - boundingArr[0][1], boundingArr[0][4] - boundingArr[0][2]);
		cv::Rect rect_2(boundingArr[1][1], boundingArr[1][2], boundingArr[1][3] - boundingArr[1][1], boundingArr[1][4] - boundingArr[1][2]);
		cv::Mat img_2 = raw_img_2(rect_1);
		cv::Mat img_1 = raw_img_1(rect_2);

		cv::imwrite("1.png", img_1);
		cv::imwrite("2.png", img_2);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = test_generatePointCloudFromDepthImage(img_1, intrinsic_n1_1);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2 = test_generatePointCloudFromDepthImage(raw_img_2, intrinsic_n1_1);

		pcl::io::savePLYFileBinary("1.ply", *cloud_1);
		pcl::io::savePLYFileBinary("2.ply", *cloud_2);
	}*/
}