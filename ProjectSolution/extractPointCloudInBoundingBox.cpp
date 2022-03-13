#include "extractPointCloudInBoundingBox.h"

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

void cutPointCloud(std::string depth_path, std::string mask_path) {

	std::vector<std::string> depth_fileNames, mask_fileNames;
	const double intrinsic_n1[4] = { 571.15928878, 571.16352329, 311.07448762, 233.73421022 };
	const double intrinsic_n2[4] = { 572.19350868, 572.19147096, 311.08874623, 228.22643626 };
	const double intrinsic_n3[4] = { 569.22935734, 569.22937915, 308.72611105, 225.53660806 };
	const double intrinsic_n4[4] = { 567.62010676, 567.62984091, 311.41553264, 225.10048896 };
	const double intrinsic_n5[4] = { 572.97665944, 572.96466485, 310.95680422, 215.36230807 };
	cv::glob(depth_path, depth_fileNames, false);
	cv::glob(mask_path, mask_fileNames, false);
	std::size_t i = 0;

	pcl::PointCloud<pcl::PointXYZ> mergeCloud;
	std::string mergeCloudPath = "D:/DATA/Research/demoData/red_bull/allPointCloud.ply";

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
			std::string toErase = "D:/DATA/Research/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		case '2':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n2);
			std::string toErase = "D:/DATA/Research/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		case '3':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n3);
			std::string toErase = "D:/DATA/Research/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		case '4':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n4);
			std::string toErase = "D:/DATA/Research/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/demoData/red_bull/pointCloud/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			mergeCloud += *cloud;

			std::cout << output << std::endl;
			break;
		}

		default:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(croppedImg, intrinsic_n5);
			std::string toErase = "D:/DATA/Research/demoData/red_bull/depth\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/demoData/red_bull/pointCloud/" + output;
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
