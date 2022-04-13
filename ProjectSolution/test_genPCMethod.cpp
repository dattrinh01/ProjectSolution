#include "test_genPCMethod.h"


std::string createPLYFileNames(std::string path, std::string toErase) {
	std::size_t pos = path.find(toErase);
	if (pos != std::string::npos)
	{
		path.erase(pos, toErase.length());
	}
	path.erase(std::prev(path.end()));
	path.erase(std::prev(path.end()));
	path.erase(std::prev(path.end()));
	path.erase(std::prev(path.end()));
	path += ".ply";
	return path;
}

void test_generate_new_point_cloud_method(std::string depth_path, std::string mask_path) {
	std::vector<std::string> depth_fileNames, mask_fileNames;
	const double intrinsic_n1[4] = { 571.15928878, 571.16352329, 311.07448762, 233.73421022 };
	const double intrinsic_n2[4] = { 572.19350868, 572.19147096, 311.08874623, 228.22643626 };
	const double intrinsic_n3[4] = { 569.22935734, 569.22937915, 308.72611105, 225.53660806 };
	const double intrinsic_n4[4] = { 567.62010676, 567.62984091, 311.41553264, 225.10048896 };
	const double intrinsic_n5[4] = { 572.97665944, 572.96466485, 310.95680422, 215.36230807 };

	const double rvec_1[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_2[3] = { 1.35616344, 2.10595134, -1.42210062 };
	const double rvec_3[3] = { 1.56657549, 1.70420951, -0.85745274 };
	const double rvec_4[3] = { 1.61963543, 1.33318051, -0.40598225 };
	const double rvec_5[3] = { 1.61106379, 1.06166743, -0.11593372 };
	const double rvec_6[3] = { 1.62660842, 1.28622777, -0.36563719 };
	const double rvec_7[3] = { 1.58353979, 1.60199123, -0.73663599 };
	const double rvec_8[3] = { 1.45545087, 1.96536236, -1.21193346 };
	const double rvec_9[3] = { 1.62285857, 1.37495105, -0.47484482 };
	const double rvec_10[3] = { 1.45590784, 1.95503687, -1.20259261 };
	const double rvec_11[3] = { 1.56728623, 1.69647319, -0.84700537 };
	const double rvec_12[3] = { 1.40849942, 2.04377181, -1.32444713 };
	const double rvec_13[3] = { 1.22845501, 2.25730994, -1.67111105 };
	const double rvec_14[3] = { 1.62762142, 1.19325915, -0.26631408 };
	const double rvec_15[3] = { 1.60673483, 0.9429037, -0.00874077 };



	const double rvec_16[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_17[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_18[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_19[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_20[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_21[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_22[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_23[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_24[3] = { 1.12134992, 1.38173224, -1.33534398 };
	const double rvec_25[3] = { 1.12134992, 1.38173224, -1.33534398 };


	cv::glob(depth_path, depth_fileNames, false);
	cv::glob(mask_path, mask_fileNames, false);
	std::size_t i = 0;

	pcl::PointCloud<pcl::PointXYZ> mergeCloud;
	std::string mergeCloudPath = "D:/DATA/Research/DrNhu/demoData/red_bull/test_new_method_pc.ply";

	for (auto const& f : depth_fileNames) {
		// Read depth image using opencv2 and generate point cloud.
		cv::Mat croppedImg;
		cv::Mat depth_img = cv::imread(depth_fileNames[i], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
		cv::Mat mask_img = cv::imread(mask_fileNames[i]);

		cv::resize(mask_img, mask_img, cv::Size(depth_img.cols, depth_img.rows), cv::INTER_LINEAR);

		// Calculate bouding box from mask image and crop image	
		double bbX, bbY, bbWidth, bbHeight;
		get_bounding_box_from_mask_image(mask_img, depth_img, bbX, bbY, bbWidth, bbHeight);
		std::cout << "x = " << bbX << "; y = " << bbY << "; width = " << bbWidth << "; height = " << bbHeight << std::endl;
		depth_img(cv::Rect(bbX, bbY, bbWidth, bbHeight)).copyTo(croppedImg);

		switch (depth_fileNames[i][34])
		{
		case '1':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n1);
			std::string toErase = "D:/DATA/Research/DrNhu/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhu/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		case '2':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n2);
			std::string toErase = "D:/DATA/Research/DrNhu/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhu/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		case '3':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n3);
			std::string toErase = "D:/DATA/Research/DrNhu/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhu/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		case '4':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n4);
			std::string toErase = "D:/DATA/Research/DrNhu/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhu/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		default:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n5);
			std::string toErase = "D:/DATA/Research/DrNhu/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhu/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}
		}

		/*pcl::visualization::CloudViewer viewer("Depth");
		viewer.showCloud(cloud);
		while(!viewer.wasStopped()) {}*/

		i++;
	}

	pcl::io::savePLYFileBinary(mergeCloudPath, mergeCloud);
}

void read_csv_file(std::string csv_path) {

}