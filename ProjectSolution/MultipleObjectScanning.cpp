#include "MultipleObjectScanning.h"

/*-----------SUPPORT FUNCTIONS-----------*/

bool naturalSorting(const std::string& a, const std::string& b)
{
	if (a.empty())
		return true;
	if (b.empty())
		return false;
	if (std::isdigit(a[0]) && !std::isdigit(b[0]))
		return true;
	if (!std::isdigit(a[0]) && std::isdigit(b[0]))
		return false;
	if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
	{
		if (std::toupper(a[0]) == std::toupper(b[0]))
			return naturalSorting(a.substr(1), b.substr(1));
		return (std::toupper(a[0]) < std::toupper(b[0]));
	}

	/*Both strings begin with digit --> parse both numbers*/
	std::istringstream issa(a);
	std::istringstream issb(b);
	int ia, ib;
	issa >> ia;
	issb >> ib;
	if (ia != ib)
		return ia < ib;

	/*Numbers are the same --> remove numbers and recurse*/
	std::string anew, bnew;
	std::getline(issa, anew);
	std::getline(issb, bnew);
	return (naturalSorting(anew, bnew));
}

void eraseSubStrings(std::string& mainString, std::string& toErase)
{
	std::size_t position = mainString.find(toErase);
	if (position != std::string::npos)
	{
		mainString.erase(position, toErase.length());
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloud(cv::Mat depth_img, const double depth_intrinsic[4])
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

bool checkSubString(std::string mainString, std::string checkString)
{
	if (strstr(mainString.c_str(), checkString.c_str()))
	{
		return true;
	}
	else
	{
		return false;
	}
}

void extractBoundingBoxFromMaskImage(cv::Mat mask_img, double& bbX, double& bbY, double& bbWidth, double& bbHeight)
{
	cv::Mat mSource_Gray, mThreshold;
	cv::cvtColor(mask_img, mSource_Gray, cv::COLOR_BGR2GRAY);
	cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);
	cv::Mat Points;
	findNonZero(mThreshold, Points);
	cv::Rect Min_Rect = boundingRect(Points);

	double x_min, x_max, y_min, y_max;
	x_min = Min_Rect.tl().x;
	y_min = Min_Rect.tl().y;
	x_max = Min_Rect.br().x;
	y_max = Min_Rect.br().y;

	bbX = (x_min + x_max) / 2.0 / mask_img.size().width;
	bbY = (y_min + y_max) / 2.0 / mask_img.size().height;
	bbWidth = (x_max - x_min) / mask_img.size().width;
	bbHeight = (y_max - y_min) / mask_img.size().height;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloudFromDepthImage(cv::Mat depth_img, const double depth_intrinsic[4])
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

void cropAndCreatePointCloud(std::string boundingBoxPath, std::string depthPath, std::string outputPath)
{
	std::ifstream inputfile(boundingBoxPath);
	double boundingArr[2][5];
	if (!inputfile.is_open())
	{
		std::cout << "Error opening file";
	}
	for (int r = 0; r < 2; r++)
	{
		for (int c = 0; c < 5; c++)
		{
			inputfile >> boundingArr[r][c];
		}
	}

	std::string checkVerticalOrHorizontal = boundingBoxPath.substr(boundingBoxPath.find(".") - 1);

	if (checkVerticalOrHorizontal == "v.txt")
	{
		boundingArr[0][1] *= 1280;
		boundingArr[0][2] *= 2048;
		boundingArr[0][3] *= 1280;
		boundingArr[0][4] *= 2048;

		boundingArr[0][1] = boundingArr[0][1] - boundingArr[0][3] / 2;
		boundingArr[0][2] = boundingArr[0][2] - boundingArr[0][4] / 2;
		boundingArr[0][3] = boundingArr[0][1] + boundingArr[0][3];
		boundingArr[0][4] = boundingArr[0][2] + boundingArr[0][4];

		boundingArr[0][1] = (boundingArr[0][1] / 1280 * 640) + 10;
		boundingArr[0][2] = boundingArr[0][2] / 2048 * 960;
		boundingArr[0][3] = boundingArr[0][3] / 1280 * 640;
		boundingArr[0][4] = boundingArr[0][4] / 2048 * 960;

		boundingArr[1][1] *= 1280;
		boundingArr[1][2] *= 2048;
		boundingArr[1][3] *= 1280;
		boundingArr[1][4] *= 2048;

		boundingArr[1][1] = boundingArr[1][1] - boundingArr[1][3] / 2;
		boundingArr[1][2] = boundingArr[1][2] - boundingArr[1][4] / 2;
		boundingArr[1][3] = boundingArr[1][1] + boundingArr[1][3];
		boundingArr[1][4] = boundingArr[1][2] + boundingArr[1][4];

		boundingArr[1][1] = boundingArr[1][1] / 1280 * 640;
		boundingArr[1][2] = boundingArr[1][2] / 2048 * 960;
		boundingArr[1][3] = boundingArr[1][3] / 1280 * 640;
		boundingArr[1][4] = boundingArr[1][4] / 2048 * 960;	

		

		cv::Mat croppedImg1, croppedImg2;
		cv::Mat depth_frame = cv::imread(depthPath, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
		depth_frame(cv::Rect(boundingArr[0][1] + 10, boundingArr[0][2],
			(boundingArr[0][3] - boundingArr[0][1]) + 10, (boundingArr[0][4] - boundingArr[0][2]) + 10)).copyTo(croppedImg1);

		depth_frame(cv::Rect(boundingArr[1][1] + 10, boundingArr[1][2],
			(boundingArr[1][3] - boundingArr[1][1]) + 10, (boundingArr[1][4] - boundingArr[1][2]) + 10)).copyTo(croppedImg2);
		
		if (boundingArr[0][0] == 0)
		{

		}

		/*cv::imwrite(boundingBoxPath + "1.png", croppedImg1);
		cv::imwrite(boundingBoxPath + "2.png", croppedImg2);
		std::cout << "Save " + boundingBoxPath + ".png" << std::endl;*/

	}
	else
	{
		boundingArr[0][1] *= 2560;
		boundingArr[0][2] *= 1024;
		boundingArr[0][3] *= 2560;
		boundingArr[0][4] *= 1024;

		boundingArr[0][1] = boundingArr[0][1] - boundingArr[0][3] / 2;
		boundingArr[0][2] = boundingArr[0][2] - boundingArr[0][4] / 2;
		boundingArr[0][3] = boundingArr[0][1] + boundingArr[0][3];
		boundingArr[0][4] = boundingArr[0][2] + boundingArr[0][4];

		boundingArr[0][1] = boundingArr[0][1] / 2560 * 1280;
		boundingArr[0][2] = boundingArr[0][2] / 1024 * 480;
		boundingArr[0][3] = boundingArr[0][3] / 2560 * 1280;
		boundingArr[0][4] = boundingArr[0][4] / 1024 * 480;


		boundingArr[1][1] *= 2560;
		boundingArr[1][2] *= 1024;
		boundingArr[1][3] *= 2560;
		boundingArr[1][4] *= 1024;

		boundingArr[1][1] = boundingArr[1][1] - boundingArr[1][3] / 2;
		boundingArr[1][2] = boundingArr[1][2] - boundingArr[1][4] / 2;
		boundingArr[1][3] = boundingArr[1][1] + boundingArr[1][3];
		boundingArr[1][4] = boundingArr[1][2] + boundingArr[1][4];

		
		boundingArr[1][1] = boundingArr[1][1] / 2560 * 1280;
		boundingArr[1][2] = boundingArr[1][2] / 1024 * 480;
		boundingArr[1][3] = boundingArr[1][3] / 2560 * 1280;
		boundingArr[1][4] = boundingArr[1][4] / 1024 * 480;

		cv::Mat croppedImg;
		cv::Mat depth_frame = cv::imread(depthPath, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
		depth_frame(cv::Rect(boundingArr[0][1], boundingArr[0][2], boundingArr[0][3], boundingArr[0][4])).copyTo(croppedImg);
		cv::imwrite(boundingBoxPath + ".png", croppedImg);
		std::cout << "Save " + boundingBoxPath + ".png" << std::endl;
	}	
}

/*-----------MAIN PROCESSING FUNCTIONS-----------*/

void datasetGeneration()
{
	/* Initialization */
	std::cout << "datasetGeneration:: Initialization" << std::endl;

	const std::string mainFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/datasetGeneration";
	const std::string inputFolder = mainFolder + "/Inputs";
	const std::string outputFolder = mainFolder + "/Outputs";
	const std::string debugFolder = mainFolder + "/Debugs";

	const std::string object_1_path = "/object_1";
	const std::string object_2_path = "/object_2";
	const std::string object_3_path = "/object_3";

	const std::string object_1_rgbPath = inputFolder + object_1_path + "/rgb/*.jpg";
	const std::string object_2_rgbPath = inputFolder + object_2_path + "/rgb/*.jpg";
	const std::string object_3_rgbPath = inputFolder + object_3_path + "/rgb/*.jpg";

	const std::string object_1_depthPath = inputFolder + object_1_path + "/depth/*.png";
	const std::string object_2_depthPath = inputFolder + object_2_path + "/depth/*.png";
	const std::string object_3_depthPath = inputFolder + object_3_path + "/depth/*.png";

	std::vector<std::string> object_1_rgbFileNames;
	std::vector<std::string> object_2_rgbFileNames;
	std::vector<std::string> object_3_rgbFileNames;

	std::vector<std::string> object_1_depthFileNames;
	std::vector<std::string> object_2_depthFileNames;
	std::vector<std::string> object_3_depthFileNames;

	std::size_t index = 0;
	bool statusReadData = true;

	/* Read data */
	std::cout << "datasetGeneration:: Read data" << std::endl;

	cv::glob(object_1_rgbPath, object_1_rgbFileNames);
	cv::glob(object_2_rgbPath, object_2_rgbFileNames);
	cv::glob(object_3_rgbPath, object_3_rgbFileNames);

	cv::glob(object_1_depthPath, object_1_depthFileNames);
	cv::glob(object_2_depthPath, object_2_depthFileNames);
	cv::glob(object_3_depthPath, object_3_depthFileNames);

	std::sort(object_1_rgbFileNames.begin(), object_1_rgbFileNames.end(), naturalSorting);
	std::sort(object_2_rgbFileNames.begin(), object_2_rgbFileNames.end(), naturalSorting);
	std::sort(object_3_rgbFileNames.begin(), object_3_rgbFileNames.end(), naturalSorting);

	std::sort(object_1_depthFileNames.begin(), object_1_depthFileNames.end(), naturalSorting);
	std::sort(object_2_depthFileNames.begin(), object_2_depthFileNames.end(), naturalSorting);
	std::sort(object_3_depthFileNames.begin(), object_3_depthFileNames.end(), naturalSorting);

	/*for (auto const& f : object_1_rgbFileNames)
	{
		cv::Mat object_1_rgb_img = cv::imread(object_1_rgbFileNames[index]);
		cv::Mat object_2_rgb_img = cv::imread(object_2_rgbFileNames[index]);
		cv::Mat object_3_rgb_img = cv::imread(object_3_rgbFileNames[index]);

		cv::Mat object_1_depth_img = cv::imread(object_1_depthFileNames[index]);
		cv::Mat object_2_depth_img = cv::imread(object_2_depthFileNames[index]);
		cv::Mat object_3_depth_img = cv::imread(object_3_depthFileNames[index]);

		if ((!object_1_rgb_img.empty()) && (!object_2_rgb_img.empty()) && (!object_3_rgb_img.empty()
			&& (!object_1_depth_img.empty()) && (!object_2_depth_img.empty()) && (!object_3_depth_img.empty())))
		{
			statusReadData = true;
		}
		else
		{
			statusReadData = false;
		}
		index++;
	}*/

	if (statusReadData == true)
	{
		std::cout << "datasetGeneration:: Read data: Success" << std::endl;

		/*Process data*/
		std::cout << "datasetGeneration:: Process data: " << std::endl;

		std::cout << "datasetGeneration:: Process data: Merge rgb images" << std::endl;

		index = 0;

		for (auto const& f : object_1_rgbFileNames)
		{
			cv::Mat object_1_rgb_img = cv::imread(object_1_rgbFileNames[index]);
			cv::Mat object_2_rgb_img = cv::imread(object_2_rgbFileNames[index]);
			cv::Mat object_3_rgb_img = cv::imread(object_3_rgbFileNames[index]);

			cv::Mat object_1_depth_img = cv::imread(object_1_depthFileNames[index]);
			cv::Mat object_2_depth_img = cv::imread(object_2_depthFileNames[index]);
			cv::Mat object_3_depth_img = cv::imread(object_3_depthFileNames[index]);

			if ((!object_1_rgb_img.empty()) && (!object_2_rgb_img.empty()) && (!object_3_rgb_img.empty()) 
				&& (!object_1_depth_img.empty()) && (!object_2_depth_img.empty()) && (!object_3_depth_img.empty()))
			{
				std::string nameFile = object_1_rgbFileNames[index].substr(object_1_rgbFileNames[index].find("\\") + 1);
				std::string toErase = ".jpg";
				eraseSubStrings(nameFile, toErase);

				cv::Mat concatenateImageRGB_object12_v, concatenateImageRGB_object23_v, concatenateImageRGB_object12_h, concatenateImageRGB_object23_h;
				cv::Mat concatenateImageDepth_object12_v, concatenateImageDepth_object23_v, concatenateImageDepth_object12_h, concatenateImageDepth_object23_h;

				cv::vconcat(object_1_rgb_img, object_2_rgb_img, concatenateImageRGB_object12_v);
				cv::vconcat(object_2_rgb_img, object_3_rgb_img, concatenateImageRGB_object23_v);
				cv::hconcat(object_1_rgb_img, object_2_rgb_img, concatenateImageRGB_object12_h);
				cv::hconcat(object_2_rgb_img, object_3_rgb_img, concatenateImageRGB_object23_h);

				cv::vconcat(object_1_depth_img, object_2_depth_img, concatenateImageDepth_object12_v);
				cv::vconcat(object_2_depth_img, object_3_depth_img, concatenateImageDepth_object23_v);
				cv::hconcat(object_1_depth_img, object_2_depth_img, concatenateImageDepth_object12_h);
				cv::hconcat(object_2_depth_img, object_3_depth_img, concatenateImageDepth_object23_h);

				std::string saveConcatenateImagePathObject12_v = outputFolder + "/Object12/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_v.png";
				std::string saveConcatenateImagePathObject23_v = outputFolder + "/Object23/rgb/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_v.png";
				std::string saveConcatenateImagePathObject12_h = outputFolder + "/Object12/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_h.png";
				std::string saveConcatenateImagePathObject23_h = outputFolder + "/Object23/rgb/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_h.png";

				std::string saveConcatenateImageDepthPathObject12_v = outputFolder + "/Object12/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_v.png";
				std::string saveConcatenateImageDepthPathObject23_v = outputFolder + "/Object23/depth/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_v.png";
				std::string saveConcatenateImageDepthPathObject12_h = outputFolder + "/Object12/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_h.png";
				std::string saveConcatenateImageDepthPathObject23_h = outputFolder + "/Object23/depth/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_h.png";

				cv::imwrite(saveConcatenateImagePathObject12_v, concatenateImageRGB_object12_v);
				cv::imwrite(saveConcatenateImagePathObject23_v, concatenateImageRGB_object23_v);
				cv::imwrite(saveConcatenateImagePathObject12_h, concatenateImageRGB_object12_h);
				cv::imwrite(saveConcatenateImagePathObject23_h, concatenateImageRGB_object23_h);

				cv::imwrite(saveConcatenateImageDepthPathObject12_v, concatenateImageDepth_object12_v);
				cv::imwrite(saveConcatenateImageDepthPathObject23_v, concatenateImageDepth_object23_v);
				cv::imwrite(saveConcatenateImageDepthPathObject12_h, concatenateImageDepth_object12_h);
				cv::imwrite(saveConcatenateImageDepthPathObject23_h, concatenateImageDepth_object23_h);

				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject12_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject23_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject12_h << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject23_h << std::endl;

				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject12_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject23_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject12_h << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject23_h << std::endl;
			}
			else
			{
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_1_rgbFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_2_rgbFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_3_rgbFileNames[index] << std::endl;

				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_1_depthFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_2_depthFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_3_depthFileNames[index] << std::endl;
			}
			index++;
		}
		/*Finalizing*/
		std::cout << "datasetGeneration:: Merge RGB and Depth image: Finalizing" << std::endl;
	}
	else
	{
		std::cout << "datasetGeneration:: Merge RGB and Depth image: Failed";
	}
	std::cout << "datasetGeneration:: Finalizing" << std::endl;

}

void detectMultipleObjects()
{
	/* Detect using Yolov5 Python */
	/* Initialization */
	std::cout << "detectMultipleObjects:: Initialization" << std::endl;

	const std::string mainFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/detectMultipleObjects";
	const std::string inputFolder = mainFolder + "/Inputs";
	const std::string outputFolder = mainFolder + "/Outputs";
	const std::string debugFolder = mainFolder + "/Debugs";

	const std::string object_1_path = "/object_1";
	const std::string object_2_path = "/object_2";
	const std::string object_3_path = "/object_3";

	const std::string object_1_rgbPath = inputFolder + object_1_path + "/rgb/*.jpg";
	const std::string object_2_rgbPath = inputFolder + object_2_path + "/rgb/*.jpg";
	const std::string object_3_rgbPath = inputFolder + object_3_path + "/rgb/*.jpg";

	const std::string object_1_maskPath = inputFolder + object_1_path + "/masks/*.pbm";
	const std::string object_2_maskPath = inputFolder + object_2_path + "/masks/*.pbm";
	const std::string object_3_maskPath = inputFolder + object_3_path + "/masks/*.pbm";

	std::vector<std::string> object_1_rgbFileNames;
	std::vector<std::string> object_2_rgbFileNames;
	std::vector<std::string> object_3_rgbFileNames;

	std::vector<std::string> object_1_maskFileNames;
	std::vector<std::string> object_2_maskFileNames;
	std::vector<std::string> object_3_maskFileNames;

	cv::Point min_coor, max_coor;
	double bbX, bbY, bbWidth, bbHeight;


	std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images" << std::endl;



	bool checkStatusReadData = false;
	std::size_t index = 0;

	/*Process data*/
	cv::glob(object_1_rgbPath, object_1_rgbFileNames);
	cv::glob(object_2_rgbPath, object_2_rgbFileNames);
	cv::glob(object_3_rgbPath, object_3_rgbFileNames);

	cv::glob(object_1_maskPath, object_1_maskFileNames);
	cv::glob(object_2_maskPath, object_2_maskFileNames);
	cv::glob(object_3_maskPath, object_3_maskFileNames);

	std::sort(object_1_rgbFileNames.begin(), object_1_rgbFileNames.end(), naturalSorting);
	std::sort(object_2_rgbFileNames.begin(), object_2_rgbFileNames.end(), naturalSorting);
	std::sort(object_3_rgbFileNames.begin(), object_3_rgbFileNames.end(), naturalSorting);

	std::sort(object_1_maskFileNames.begin(), object_1_maskFileNames.end(), naturalSorting);
	std::sort(object_2_maskFileNames.begin(), object_2_maskFileNames.end(), naturalSorting);
	std::sort(object_3_maskFileNames.begin(), object_3_maskFileNames.end(), naturalSorting);

	for (auto const& f : object_1_rgbFileNames)
	{
		cv::Mat rgb_obj_1_img = cv::imread(object_1_rgbFileNames[index]);
		cv::Mat rgb_obj_2_img = cv::imread(object_2_rgbFileNames[index]);
		cv::Mat rgb_obj_3_img = cv::imread(object_3_rgbFileNames[index]);

		cv::Mat mask_obj_1_img = cv::imread(object_1_maskFileNames[index]);
		cv::Mat mask_obj_2_img = cv::imread(object_2_maskFileNames[index]);
		cv::Mat mask_obj_3_img = cv::imread(object_3_maskFileNames[index]);

		if ((!rgb_obj_1_img.empty()) && (!rgb_obj_2_img.empty()) && (!rgb_obj_3_img.empty()
			&& (!mask_obj_1_img.empty()) && (!mask_obj_2_img.empty()) && (!mask_obj_3_img.empty())))
		{
			checkStatusReadData = true;
		}
		else
		{
			checkStatusReadData = false;
		}
		index++;
	}

	if (checkStatusReadData == true)
	{
		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Success" << std::endl;
	}
	else
	{
		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Failed" << std::endl;
	}

	index = 0;

	for (auto const& f : object_1_rgbFileNames)
	{
		cv::Mat rgb_obj_1_img = cv::imread(object_1_rgbFileNames[index]);
		cv::Mat rgb_obj_2_img = cv::imread(object_2_rgbFileNames[index]);
		cv::Mat rgb_obj_3_img = cv::imread(object_3_rgbFileNames[index]);

		cv::Mat mask_obj_1_img = cv::imread(object_1_maskFileNames[index]);
		cv::Mat mask_obj_2_img = cv::imread(object_2_maskFileNames[index]);
		cv::Mat mask_obj_3_img = cv::imread(object_3_maskFileNames[index]);

		std::string nameFile;
		std::string toErase;
		std::string savePathName;
		std::ofstream boundingBoxCoordinate;

		/*------------------------------------------------------------------------------------*/
		nameFile = "";
		toErase = "";
		savePathName = "";

		nameFile = object_1_rgbFileNames[index].substr(object_1_rgbFileNames[index].find("//") + 1);
		toErase = ".jpg";
		eraseSubStrings(nameFile, toErase);

		extractBoundingBoxFromMaskImage(mask_obj_1_img, bbX, bbY, bbWidth, bbHeight);
		savePathName = outputFolder + "/class_0/" + nameFile + "_obj_1.txt";
		boundingBoxCoordinate.open(savePathName);
		boundingBoxCoordinate << "0 " + std::to_string(bbX) + " " + std::to_string(bbY) + " " +
			std::to_string(bbWidth) + " " + std::to_string(bbHeight);
		boundingBoxCoordinate.close();
		cv::imwrite(outputFolder + "/class_0/" + nameFile + "_obj_1.png", rgb_obj_1_img);

		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Save image: "
			<< outputFolder + "/class_0/" + nameFile + "_obj_1.png" << std::endl;

		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Write bounding box: "
			<< savePathName << std::endl;

		/*------------------------------------------------------------------------------------*/

		nameFile = "";
		toErase = "";
		savePathName = "";

		nameFile = object_2_rgbFileNames[index].substr(object_2_rgbFileNames[index].find("//") + 1);
		toErase = ".jpg";
		eraseSubStrings(nameFile, toErase);

		extractBoundingBoxFromMaskImage(mask_obj_2_img, bbX, bbY, bbWidth, bbHeight);
		savePathName = outputFolder + "/class_1/" + nameFile + "_obj_2.txt";
		boundingBoxCoordinate.open(savePathName);
		boundingBoxCoordinate << "1 " + std::to_string(bbX) + " " + std::to_string(bbY) + " " +
			std::to_string(bbWidth) + " " + std::to_string(bbHeight);
		boundingBoxCoordinate.close();
		cv::imwrite(outputFolder + "/class_1/" + nameFile + "_obj_2.png", rgb_obj_2_img);

		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Save image: "
			<< outputFolder + "/class_1/" + nameFile + "_obj_2.png" << std::endl;

		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Write bounding box: "
			<< savePathName << std::endl;

		/*------------------------------------------------------------------------------------*/
		nameFile = "";
		toErase = "";
		savePathName = "";

		nameFile = object_3_rgbFileNames[index].substr(object_3_rgbFileNames[index].find("//") + 1);
		toErase = ".jpg";
		eraseSubStrings(nameFile, toErase);

		extractBoundingBoxFromMaskImage(mask_obj_3_img, bbX, bbY, bbWidth, bbHeight);
		savePathName = outputFolder + "/class_1/" + nameFile + "_obj_3.txt";
		boundingBoxCoordinate.open(savePathName);

		boundingBoxCoordinate << "1 " + std::to_string(bbX) + " " + std::to_string(bbY) + " " +
			std::to_string(bbWidth) + " " + std::to_string(bbHeight);
		boundingBoxCoordinate.close();
		cv::imwrite(outputFolder + "/class_1/" + nameFile + "_obj_3.png", rgb_obj_3_img);

		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Save image: "
			<< outputFolder + "/class_1/" + nameFile + "_obj_3.png" << std::endl;

		std::cout << "detectMultipleObjects:: Process data: Extract bounding box from mask images: Write bounding box: "
			<< savePathName << std::endl;
		index++;

	}

	/*Finalizing*/
	std::cout << "detectMultipleObjects:: Finalizing" << std::endl;

}

void cutPointCloudOfDetectObjects()
{
	std::cout << "cutPointCloudOfDetectObjects:: Initialization" << std::endl;

	const std::string mainFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/cutPointCloudOfDetectObjects";

	const std::string inputFolder = mainFolder + "/Inputs";
	const std::string object12PathInputBoundingBox = inputFolder + "/obj12/*.txt";
	const std::string object23PathInputBoundingBox = inputFolder + "/obj23/*.txt";

	const std::string object12PathInputImage = inputFolder + "/obj12/*.png";
	const std::string object23PathInputImage = inputFolder + "/obj23/*.png";


	const std::string outputPointCloudFolder = mainFolder + "/Outputs";
	const std::string object1PathOutput = outputPointCloudFolder + "/obj1";
	const std::string object2PathOutput = outputPointCloudFolder + "/obj2";
	const std::string object3PathOutput = outputPointCloudFolder + "/obj3";

	const std::string debugFolder = mainFolder + "/Debugs";

	const double intrinsic_n1[4] = { 571.15928878, 571.16352329, 311.07448762, 233.73421022 };
	const double intrinsic_n2[4] = { 572.19350868, 572.19147096, 311.08874623, 228.22643626 };
	const double intrinsic_n3[4] = { 569.22935734, 569.22937915, 308.72611105, 225.53660806 };
	const double intrinsic_n4[4] = { 567.62010676, 567.62984091, 311.41553264, 225.10048896 };
	const double intrinsic_n5[4] = { 572.97665944, 572.96466485, 310.95680422, 215.36230807 };

	std::vector<std::string> boundingBox12Path;
	std::vector<std::string> boundingBox23Path;

	std::vector<std::string> depthImage12Path;
	std::vector<std::string> depthImage23Path;

	std::size_t index = 0;
	int count = 0;

	std::string nameBoundingBoxFile = "";
	std::string nameDepthImageFile = "";
	std::string toEraseTXT = ".txt";
	std::string toErasePNG = ".png";

	std::cout << "cutPointCloudOfDetectObjects::Process data: Read data" << std::endl;

	cv::glob(object12PathInputBoundingBox, boundingBox12Path);
	cv::glob(object23PathInputBoundingBox, boundingBox23Path);

	cv::glob(object12PathInputImage, depthImage12Path);
	cv::glob(object23PathInputImage, depthImage23Path);

	std::sort(boundingBox12Path.begin(), boundingBox12Path.end(), naturalSorting);
	std::sort(boundingBox23Path.begin(), boundingBox23Path.end(), naturalSorting);

	std::sort(depthImage12Path.begin(), depthImage12Path.end(), naturalSorting);
	std::sort(depthImage23Path.begin(), depthImage23Path.end(), naturalSorting);

	std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " << std::endl;
	for (auto const& f : depthImage12Path)
	{
		nameBoundingBoxFile = boundingBox12Path[index];
		eraseSubStrings(nameBoundingBoxFile, toEraseTXT);

		nameDepthImageFile = depthImage12Path[index];
		eraseSubStrings(nameDepthImageFile, toErasePNG);

		if (nameBoundingBoxFile == nameDepthImageFile) {
			/*Read txt bounding box*/
			/*Read bounding box while merge image*/
			/*Cut depth frame from bounding box*/
			/*Check NP_n to choose intrinsic matrix*/
			/*Generate point cloud using correct intrinsics matrix*/
			/*Save point cloud*/
			std::string checkViewObject = boundingBox12Path[index].substr(boundingBox12Path[index].find("NP")).substr(0, 3);
			/*std::cout << checkViewObject << std::endl;*/

			if (checkViewObject == "NP1")
			{
				/* Read bounding box from YOLOv5*/
				std::ifstream inputfile(boundingBox12Path[index]);
				double boundingArr[2][5];
				if (!inputfile.is_open())
				{
					std::cout << "Error opening file";
				}
				for (int r = 0; r < 2; r++)
				{
					for (int c = 0; c < 5; c++)
					{
						inputfile >> boundingArr[r][c];
					}
				}

				for (int i = 0; i < 2; i++)
				{
					if (boundingArr[i][0] == 0)
					{
						std::string checkVerticalOrHorizontal = boundingBox12Path[index].substr(boundingBox12Path[index].find(".") - 1);
						std::string test_output_path = boundingBox12Path[index].substr(boundingBox12Path[index].find("NP"));
						std::string savePath_1 = test_output_path.substr(0, test_output_path.find("_merge_"));
						std::string savePath_2 = test_output_path.substr(test_output_path.find("_merge_") + 7);
						std::cout << savePath_1 << std::endl;
						std::cout << savePath_2 << std::endl;
						if (checkVerticalOrHorizontal == "v.txt")
						{
							boundingArr[i][1] *= 1280;
							boundingArr[i][2] *= 2048;
							boundingArr[i][3] *= 1280;
							boundingArr[i][4] *= 2048;

							boundingArr[i][1] = boundingArr[i][1] - boundingArr[i][3] / 2;
							boundingArr[i][2] = boundingArr[i][2] - boundingArr[i][4] / 2;
							boundingArr[i][3] = boundingArr[i][1] + boundingArr[i][3];
							boundingArr[i][4] = boundingArr[i][2] + boundingArr[i][4];

							boundingArr[i][1] = (boundingArr[i][1] / 1280 * 640) + 10;
							boundingArr[i][2] = boundingArr[i][2] / 2048 * 960;
							boundingArr[i][3] = boundingArr[i][3] / 1280 * 640;
							boundingArr[i][4] = boundingArr[i][4] / 2048 * 960;

							cv::Mat croppedImg;
							cv::Mat depth_frame = cv::imread(nameDepthImageFile + ".png", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
							depth_frame(cv::Rect(boundingArr[i][1] + 10, boundingArr[i][2],
								(boundingArr[i][3] - boundingArr[i][1]) + 10, (boundingArr[i][4] - boundingArr[i][2]) + 10)).copyTo(croppedImg);

							cv::imwrite(object1PathOutput + "/" + savePath_1 + "_v.png", croppedImg);
							
						}
						else if (checkVerticalOrHorizontal == "h.txt")
						{
							boundingArr[i][1] *= 2560;
							boundingArr[i][2] *= 1024;
							boundingArr[i][3] *= 2560;
							boundingArr[i][4] *= 1024;

							boundingArr[i][1] = boundingArr[1][1] - boundingArr[1][3] / 2;
							boundingArr[i][2] = boundingArr[1][2] - boundingArr[1][4] / 2;
							boundingArr[i][3] = boundingArr[1][1] + boundingArr[1][3];
							boundingArr[i][4] = boundingArr[1][2] + boundingArr[1][4];


							boundingArr[i][1] = boundingArr[1][1] / 2560 * 1280;
							boundingArr[i][2] = boundingArr[1][2] / 1024 * 480;
							boundingArr[i][3] = boundingArr[1][3] / 2560 * 1280;
							boundingArr[i][4] = boundingArr[1][4] / 1024 * 480;

							cv::Mat croppedImg;
							cv::Mat depth_frame = cv::imread(nameDepthImageFile + ".png", cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
							depth_frame(cv::Rect(boundingArr[i][1] + 10, boundingArr[i][2],
								(boundingArr[i][3] - boundingArr[i][1]) + 10, (boundingArr[i][4] - boundingArr[i][2]) + 10)).copyTo(croppedImg);

							cv::imwrite(object1PathOutput + "/" + savePath_2 + ".png", croppedImg);
						}

						
					}
					else if (boundingArr[i][0] == 1)
					{
						std::string checkVerticalOrHorizontal = boundingBox12Path[index].substr(boundingBox12Path[index].find(".") - 1);
						std::string test_output_path = boundingBox12Path[index].substr(boundingBox12Path[index].find("NP"));
						std::string savePath_1 = test_output_path.substr(0, test_output_path.find("_merge_"));
						std::string savePath_2 = test_output_path.substr(test_output_path.find("_merge_") + 7);
						std::cout << savePath_1 << std::endl;
						std::cout << savePath_2 << std::endl;
						if (checkVerticalOrHorizontal == "v.txt")
						{
							boundingArr[i][1] *= 1280;
							boundingArr[i][2] *= 2048;
							boundingArr[i][3] *= 1280;
							boundingArr[i][4] *= 2048;

							boundingArr[i][1] = boundingArr[i][1] - boundingArr[i][3] / 2;
							boundingArr[i][2] = boundingArr[i][2] - boundingArr[i][4] / 2;
							boundingArr[i][3] = boundingArr[i][1] + boundingArr[i][3];
							boundingArr[i][4] = boundingArr[i][2] + boundingArr[i][4];

							boundingArr[i][1] = (boundingArr[i][1] / 1280 * 640) + 10;
							boundingArr[i][2] = boundingArr[i][2] / 2048 * 960;
							boundingArr[i][3] = boundingArr[i][3] / 1280 * 640;
							boundingArr[i][4] = boundingArr[i][4] / 2048 * 960;
						}
						else if (checkVerticalOrHorizontal == "h.txt")
						{
							boundingArr[i][1] *= 1280;
							boundingArr[i][2] *= 2048;
							boundingArr[i][3] *= 1280;
							boundingArr[i][4] *= 2048;

							boundingArr[i][1] = boundingArr[i][1] - boundingArr[i][3] / 2;
							boundingArr[i][2] = boundingArr[i][2] - boundingArr[i][4] / 2;
							boundingArr[i][3] = boundingArr[i][1] + boundingArr[i][3];
							boundingArr[i][4] = boundingArr[i][2] + boundingArr[i][4];

							boundingArr[i][1] = (boundingArr[i][1] / 1280 * 1280) + 10;
							boundingArr[i][2] = boundingArr[i][2] / 2048 * 480;
							boundingArr[i][3] = boundingArr[i][3] / 1280 * 1280;
							boundingArr[i][4] = boundingArr[i][4] / 2048 * 480;
						}
					}
				}
			}			
		}
		index++;
	}
}

/*-----------MAIN FUNCTIONS-----------*/

void mainFunction()
{
	/*Initialization*/
	std::cout << "mainFunction:: Initialization" << std::endl;

	/*Input data*/
	std::cout << "mainFunction:: Input data" << std::endl;

	/*Process data*/
	std::cout << "mainFunction:: Process data" << std::endl;
	datasetGeneration();
	/*cutPointCloudOfDetectObjects();*/
	/*Finalizing*/
	std::cout << "mainFunction:: Finalizing" << std::endl;
}