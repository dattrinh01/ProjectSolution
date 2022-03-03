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

	for (auto const& f : depth_fileNames) {	
		// Read depth image using opencv2 and generate point cloud.
		cv::Mat depth_img = cv::imread(depth_fileNames[i], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
		cv::Mat mask_img = cv::imread(mask_fileNames[i]);

		// Calculate bouding box from mask image and crop image
		/*int xMin, xMax, yMin, yMax;
		get_bounding_box_from_mask_image(mask_img, xMin, yMin, xMax, yMax);

		depth_img = depth_img(cv::Range(round(yMin * 640 / 1280), int(yMax * 640 / 1280)), cv::Range(int(xMin * 480 / 1024), int(xMax * 480 / 1024)));
		cv::imwrite("depth_crop/" + std::to_string(i) +".png", depth_img);*/
		
		// Choose intrinsic matrix for images                                                      
		std::string check = depth_fileNames[i].substr(53, 54);
		char camera_num = check[0];

		switch (camera_num)
		{
		case '1':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(depth_img, mask_img, intrinsic_n1);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			
			std::cout << output << std::endl;
			break;
		}

		case '2':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(depth_img, mask_img, intrinsic_n2);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}

		case '3':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(depth_img, mask_img, intrinsic_n3);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}

		case '4':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(depth_img, mask_img, intrinsic_n4);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}

		default:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloud(depth_img, mask_img, intrinsic_n5);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}
		}
	
		/*pcl::visualization::CloudViewer viewer("Depth");
		viewer.showCloud(cloud);
		while(!viewer.wasStopped()) {}*/
		
		i++;
	}

}
