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
	cv::findNonZero(mThreshold, Points);
	cv::Rect Min_Rect = boundingRect(Points);
	cv::Rect maskBoxes = cv::Rect(Min_Rect.tl().x, Min_Rect.tl().y, Min_Rect.br().x - Min_Rect.tl().x, Min_Rect.br().y - Min_Rect.tl().y);
	
	bbX = maskBoxes.x;
	bbY = maskBoxes.y;
	bbWidth = maskBoxes.width;
	bbHeight = maskBoxes.height;
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
	bool statusReadData = false;

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

	for (auto const& f : object_1_rgbFileNames)
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
	}

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

			if ((!object_1_rgb_img.empty()) && (!object_2_rgb_img.empty()) && (!object_3_rgb_img.empty()))
			{
				std::string nameFile = object_1_rgbFileNames[index].substr(object_1_rgbFileNames[index].find("\\") + 1);
				std::string toErase = ".jpg";
				eraseSubStrings(nameFile, toErase);

				cv::Mat concatenateImage_object12_v, concatenateImage_object23_v, concatenateImage_object12_h, concatenateImage_object23_h;
				cv::vconcat(object_1_rgb_img, object_2_rgb_img, concatenateImage_object12_v);
				cv::vconcat(object_2_rgb_img, object_3_rgb_img, concatenateImage_object23_v);
				cv::hconcat(object_1_rgb_img, object_2_rgb_img, concatenateImage_object12_h);
				cv::hconcat(object_2_rgb_img, object_3_rgb_img, concatenateImage_object23_h);

				std::string saveConcatenateImagePathObject12_v = outputFolder + "/Object12/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_v.png";
				std::string saveConcatenateImagePathObject23_v = outputFolder + "/Object23/rgb/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_v.png";
				std::string saveConcatenateImagePathObject12_h = outputFolder + "/Object12/rgb/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_h.png";
				std::string saveConcatenateImagePathObject23_h = outputFolder + "/Object23/rgb/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_h.png";

				cv::imwrite(saveConcatenateImagePathObject12_v, concatenateImage_object12_v);
				cv::imwrite(saveConcatenateImagePathObject23_v, concatenateImage_object23_v);
				cv::imwrite(saveConcatenateImagePathObject12_h, concatenateImage_object12_h);
				cv::imwrite(saveConcatenateImagePathObject23_h, concatenateImage_object23_h);

				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject12_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject23_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject12_h << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject23_h << std::endl;
			}
			else
			{
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_1_rgbFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_2_rgbFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Failed: " << object_3_rgbFileNames[index] << std::endl;
			}
			index++;
		}

		index = 0;

		std::cout << "datasetGeneration:: Process data: Merge depth images" << std::endl;

		for (auto const& f : object_1_depthFileNames)
		{
			cv::Mat object_1_depth_img = cv::imread(object_1_depthFileNames[index]);
			cv::Mat object_2_depth_img = cv::imread(object_2_depthFileNames[index]);
			cv::Mat object_3_depth_img = cv::imread(object_3_depthFileNames[index]);

			if ((!object_1_depth_img.empty()) && (!object_2_depth_img.empty()) && (!object_3_depth_img.empty()))
			{
				std::string nameFile = object_1_depthFileNames[index].substr(object_1_depthFileNames[index].find("\\") + 1);
				std::string toErase = ".png";
				eraseSubStrings(nameFile, toErase);

				cv::Mat concatenateImage_object12_v, concatenateImage_object23_v, concatenateImage_object12_h, concatenateImage_object23_h;

				cv::vconcat(object_1_depth_img, object_2_depth_img, concatenateImage_object12_v);
				cv::vconcat(object_2_depth_img, object_3_depth_img, concatenateImage_object23_v);

				cv::hconcat(object_1_depth_img, object_2_depth_img, concatenateImage_object12_h);
				cv::hconcat(object_2_depth_img, object_3_depth_img, concatenateImage_object23_h);

				std::string saveConcatenateImagePathObject12_v = outputFolder + "/Object12/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_v.png";
				std::string saveConcatenateImagePathObject23_v = outputFolder + "/Object23/depth/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_v.png";

				std::string saveConcatenateImagePathObject12_h = outputFolder + "/Object12/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_h.png";
				std::string saveConcatenateImagePathObject23_h = outputFolder + "/Object23/depth/" + nameFile + "_obj2_merge_" + nameFile + "_obj3_h.png";

				cv::imwrite(saveConcatenateImagePathObject12_v, concatenateImage_object12_v);
				cv::imwrite(saveConcatenateImagePathObject23_v, concatenateImage_object23_v);
				cv::imwrite(saveConcatenateImagePathObject12_h, concatenateImage_object23_h);
				cv::imwrite(saveConcatenateImagePathObject23_h, concatenateImage_object23_h);

				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImagePathObject12_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImagePathObject23_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImagePathObject12_h << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImagePathObject23_h << std::endl;
			}
			else
			{
				std::cout << "datasetGeneration:: Process data: Merge depth images: Failed: " << object_1_depthFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Failed: " << object_2_depthFileNames[index] << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Failed: " << object_3_depthFileNames[index] << std::endl;
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
	std::cout << "detectMultipleObjects:: Finalizing" << std::endl;

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

		nameFile = object_1_rgbFileNames[index].substr(object_1_rgbFileNames[index].find("\\") + 1);
		toErase = ".jpg";
		eraseSubStrings(nameFile, toErase);

		extractBoundingBoxFromMaskImage(mask_obj_1_img, bbX, bbY, bbWidth, bbHeight);
		savePathName = outputFolder + "/class_0/" + nameFile + "_obj_1.txt";
		boundingBoxCoordinate.open(savePathName);
		boundingBoxCoordinate << "0 " + std::to_string(bbX / rgb_obj_1_img.rows) + " " + std::to_string(bbY / rgb_obj_1_img.cols) + " " +
			std::to_string(bbWidth / rgb_obj_1_img.rows) + " " + std::to_string(bbHeight / rgb_obj_1_img.cols);
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

		nameFile = object_2_rgbFileNames[index].substr(object_2_rgbFileNames[index].find("\\") + 1);
		toErase = ".jpg";
		eraseSubStrings(nameFile, toErase);

		extractBoundingBoxFromMaskImage(mask_obj_2_img, bbX, bbY, bbWidth, bbHeight);
		savePathName = outputFolder + "/class_1/" + nameFile + "_obj_2.txt";
		boundingBoxCoordinate.open(savePathName);
		boundingBoxCoordinate << "1 " + std::to_string(bbX / rgb_obj_2_img.rows) + " " + std::to_string(bbY / rgb_obj_2_img.cols) + " " +
			std::to_string(bbWidth / rgb_obj_2_img.rows) + " " + std::to_string(bbHeight / rgb_obj_2_img.cols);
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

		nameFile = object_3_rgbFileNames[index].substr(object_3_rgbFileNames[index].find("\\") + 1);
		toErase = ".jpg";
		eraseSubStrings(nameFile, toErase);

		extractBoundingBoxFromMaskImage(mask_obj_3_img, bbX, bbY, bbWidth, bbHeight);
		savePathName = outputFolder  + "/class_1/" + nameFile + "_obj_3.txt";
		boundingBoxCoordinate.open(savePathName);

		boundingBoxCoordinate << "1 " + std::to_string(bbX / rgb_obj_3_img.rows) + " " + std::to_string(bbY / rgb_obj_3_img.cols) + " " +
			std::to_string(bbWidth / rgb_obj_3_img.rows) + " " + std::to_string(bbHeight / rgb_obj_3_img.cols);
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

/*-----------MAIN FUNCTIONS-----------*/

void mainFunction()
{
	/*Initialization*/
	std::cout << "mainFunction:: Initialization" << std::endl;

	/*Input data*/
	std::cout << "mainFunction:: Input data" << std::endl;

	/*Process data*/
	std::cout << "mainFunction:: Process data" << std::endl;
	/*datasetGeneration();*/
	detectMultipleObjects();
	/*Finalizing*/
	std::cout << "mainFunction:: Finalizing" << std::endl;
}
