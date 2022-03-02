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
	const double intrinsic_n1[4] = { 3571.26254, 3571.26497, 2120.49682, 1466.54704 };
	const double intrinsic_n2[4] = { 3505.30944, 3505.31001, 2094.64797, 1383.66423 };
	const double intrinsic_n3[4] = { 3547.98527, 3547.98210, 2120.29662, 1418.31236 };
	const double intrinsic_n4[4] = { 3550.47318, 3550.46805, 2086.34104, 1374.67843 };
	const double intrinsic_n5[4] = { 3528.88225, 3528.88081, 2138.76913, 1403.53065 };
	cv::glob(depth_path, depth_fileNames, false);
	cv::glob(mask_path, mask_fileNames, false);
	std::size_t i = 0;

	for (auto const& f : depth_fileNames) {	
		// Read depth image using opencv2 and generate point cloud.
		cv::Mat depth_img = cv::imread(depth_fileNames[i], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
		//depth_img.convertTo(depth_img, CV_32F);
		cv::Mat mask_img = cv::imread(mask_fileNames[i]);
		// Calculate bouding box from mask image and crop image
		int xMin, xMax, yMin, yMax;
		get_bounding_box_from_mask_image(mask_img, xMin, yMin, xMax, yMax);
		depth_img = depth_img(cv::Range(int(yMin * 0.46875 - 10), int(yMax * 0.46875 + 20)), cv::Range(int(xMin * 0.5 - 10), int(xMax * 0.5 + 20)));
		//depth_img = depth_img(cv::Range(int(yMin * 0.46875), int(yMax * 0.46875)), cv::Range(int(xMin * 0.5), int(xMax * 0.5)));
		cv::imwrite("depth_crop/" + std::to_string(i) +".png", depth_img);
		// Choose intrinsic matrix for images
		std::string check = depth_fileNames[i].substr(53, 54);
		char camera_num = check[0];

		switch (camera_num)
		{
		case '1':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generate_point_cloud(depth_img, intrinsic_n1);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);
			
			std::cout << output << std::endl;
			break;
		}

		case '2':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generate_point_cloud(depth_img, intrinsic_n2);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}

		case '3':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generate_point_cloud(depth_img, intrinsic_n3);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}

		case '4':
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generate_point_cloud(depth_img, intrinsic_n4);
			std::string toErase = "D:/DATA/Research/DrNhuResearch/test_data/depth_png\\";
			std::string output = createPLYFileNames(depth_fileNames[i], toErase);
			std::string writePath = "D:/DATA/Research/DrNhuResearch/test_data/ply_data/" + output;
			pcl::io::savePLYFileBinary(writePath, *cloud);

			std::cout << output << std::endl;
			break;
		}

		default:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generate_point_cloud(depth_img, intrinsic_n5);
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
