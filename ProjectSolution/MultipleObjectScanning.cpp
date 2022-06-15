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

pcl::PointCloud<pcl::PointXYZ> convertEigenMatrixXdToPCLCloud(const Eigen::MatrixXd& inputMatrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
	outputCloud->points.resize(inputMatrix.rows());

	outputCloud->height = 1;
	outputCloud->width = inputMatrix.rows();

	for (unsigned int i = 0; i < outputCloud->points.size(); i++) {
		pcl::PointXYZ point;
		point.x = inputMatrix(i, 0);
		point.y = inputMatrix(i, 1);
		point.z = inputMatrix(i, 2);
		outputCloud->points[i] = point;
	}
	return *outputCloud;
}

Eigen::MatrixXd convertPCLCloudToEigenMatrixXd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud)
{
	Eigen::MatrixXd outputBuffer(inputCloud->points.size(), 3);
	for (unsigned int i = 0; i < outputBuffer.rows(); i++) {
		Eigen::RowVector3d point = Eigen::RowVector3d(inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z);
		outputBuffer.row(i) = point;
	}
	return outputBuffer;
}

Eigen::MatrixXd transformPointsWithTransformMatrix(const Eigen::MatrixXd& inputVertices, const Eigen::Matrix4f& transformMatrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
	Eigen::MatrixXd outputMatrix;
	*sourceCloud = convertEigenMatrixXdToPCLCloud(inputVertices);
	pcl::transformPointCloud(*sourceCloud, *transformedCloud, transformMatrix);
	outputMatrix = convertPCLCloudToEigenMatrixXd(transformedCloud);
	return outputMatrix;
}

Eigen::Matrix4f transformVerticesFromPointToPoint(const Eigen::MatrixXd& targetVertices, const Eigen::Vector3d fromPoint, const Eigen::Vector3d toPoint, Eigen::MatrixXd& outPoints)
{
	Eigen::MatrixXd outputBuffer;
	Eigen::Vector3d diffVector = (toPoint - fromPoint);
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 3) = diffVector.x(); transform(1, 3) = diffVector.y(); transform(2, 3) = diffVector.z();
	outputBuffer = transformPointsWithTransformMatrix(targetVertices, transform);
	outPoints = outputBuffer;
	return transform;
}



void transformPointCloudToOriginal(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud) 
{
	const Eigen::MatrixXd cloudMatrix = convertPCLCloudToEigenMatrixXd(inCloud);
	std::cout << "Initial targetVertical --------------------" << std::endl;
	Eigen::MatrixXd targetVertical = cloudMatrix;
	std::cout << "Initial fromPoints --------------------" << std::endl;
	Eigen::Vector3d fromPoints = cloudMatrix.colwise().mean();
	std::cout << "Initial toPoints --------------------" << std::endl;
	Eigen::Vector3d toPoints(0.0, 0.0, 0.0);
	std::cout << "Initial outPoints --------------------" << std::endl;
	Eigen::MatrixXd outPoints;
	std::cout << "Calculate transformation matrix ---------------" << std::endl;
	Eigen::Matrix4f transformMat = transformVerticesFromPointToPoint(targetVertical, fromPoints, toPoints, outPoints);

	*outCloud = convertEigenMatrixXdToPCLCloud(outPoints);
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

				cv::Mat concatenateImageRGB_object12_v, concatenateImageRGB_object13_v, concatenateImageRGB_object12_h, concatenateImageRGB_object13_h;
				cv::Mat concatenateImageDepth_object12_v, concatenateImageDepth_object13_v, concatenateImageDepth_object12_h, concatenateImageDepth_object13_h;

				cv::vconcat(object_1_rgb_img, object_2_rgb_img, concatenateImageRGB_object12_v);
				cv::vconcat(object_1_rgb_img, object_3_rgb_img, concatenateImageRGB_object13_v);
				cv::hconcat(object_1_rgb_img, object_2_rgb_img, concatenateImageRGB_object12_h);
				cv::hconcat(object_1_rgb_img, object_3_rgb_img, concatenateImageRGB_object13_h);

				cv::vconcat(object_1_depth_img, object_2_depth_img, concatenateImageDepth_object12_v);
				cv::vconcat(object_1_depth_img, object_3_depth_img, concatenateImageDepth_object13_v);
				cv::hconcat(object_1_depth_img, object_2_depth_img, concatenateImageDepth_object12_h);
				cv::hconcat(object_1_depth_img, object_3_depth_img, concatenateImageDepth_object13_h);

				std::string saveConcatenateImagePathObject12_v = outputFolder + "/Object12/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_v.png";
				std::string saveConcatenateImagePathObject13_v = outputFolder + "/Object13/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj3_v.png";
				std::string saveConcatenateImagePathObject12_h = outputFolder + "/Object12/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_h.png";
				std::string saveConcatenateImagePathObject13_h = outputFolder + "/Object13/rgb/" + nameFile + "_obj1_merge_" + nameFile + "_obj3_h.png";

				std::string saveConcatenateImageDepthPathObject12_v = outputFolder + "/Object12/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_v.png";
				std::string saveConcatenateImageDepthPathObject13_v = outputFolder + "/Object13/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj3_v.png";
				std::string saveConcatenateImageDepthPathObject12_h = outputFolder + "/Object12/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj2_h.png";
				std::string saveConcatenateImageDepthPathObject13_h = outputFolder + "/Object13/depth/" + nameFile + "_obj1_merge_" + nameFile + "_obj3_h.png";

				cv::imwrite(saveConcatenateImagePathObject12_v, concatenateImageRGB_object12_v);
				cv::imwrite(saveConcatenateImagePathObject13_v, concatenateImageRGB_object13_v);
				cv::imwrite(saveConcatenateImagePathObject12_h, concatenateImageRGB_object12_h);
				cv::imwrite(saveConcatenateImagePathObject13_h, concatenateImageRGB_object13_h);

				cv::imwrite(saveConcatenateImageDepthPathObject12_v, concatenateImageDepth_object12_v);
				cv::imwrite(saveConcatenateImageDepthPathObject13_v, concatenateImageDepth_object13_v);
				cv::imwrite(saveConcatenateImageDepthPathObject12_h, concatenateImageDepth_object12_h);
				cv::imwrite(saveConcatenateImageDepthPathObject13_h, concatenateImageDepth_object13_h);

				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject12_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject13_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject12_h << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge rgb images: Save concatenate image: " << saveConcatenateImagePathObject13_h << std::endl;

				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject12_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject13_v << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject12_h << std::endl;
				std::cout << "datasetGeneration:: Process data: Merge depth images: Save concatenate image: " << saveConcatenateImageDepthPathObject13_h << std::endl;
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


	/*
	Replace 3_shape1 to 2_shape1
	Replace /obj12_h and /obj12_v to /obj13_h and /obj13_v
	Replace const std::string object_2_path = "/object_2" to const std::string object_2_path = "/object_3";
	*/
	std::cout << "cutPointCloudOfDetectObjects:: Initialization" << std::endl;

	const std::string mainMergeFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/cutPointCloudOfDetectObjects";

	const std::string inputMergeFolder = mainMergeFolder + "/Inputs";
	const std::string object12PathInputBoundingBox_h = inputMergeFolder + "/obj12_h/*.txt";
	const std::string object12PathInputBoundingBox_v = inputMergeFolder + "/obj12_v/*.txt";
	const std::string outputPointCloudFolder = mainMergeFolder + "/Outputs";
	const std::string shape0PathOutput = outputPointCloudFolder + "/shape_0";
	const std::string shape1PathOutput = outputPointCloudFolder + "/shape_1";

	const std::string debugMergeFolder = mainMergeFolder + "/Debugs";


	const std::string mainRawImageFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/datasetGeneration";
	const std::string inputRawImageFolder = mainRawImageFolder + "/Inputs";
	const std::string outputFolder = mainRawImageFolder + "/Outputs";
	const std::string debugFolder = mainRawImageFolder + "/Debugs";

	const std::string object_1_path = "/object_1";
	const std::string object_2_path = "/object_2";

	const std::string object_1_depthPath = inputRawImageFolder + object_1_path + "/depth/*.png";
	const std::string object_2_depthPath = inputRawImageFolder + object_2_path + "/depth/*.png";

	/*const double intrinsic_n1[4] = { 570.31691719, 570.31988743, 314.9278689, 228.59060364 };
	const double intrinsic_n2[4] = { 572.31327882, 572.31155976, 314.34439596, 228.35836425 };
	const double intrinsic_n3[4] = { 568.84098602, 568.84158184, 313.36329231, 224.84618475 };
	const double intrinsic_n4[4] = { 567.32716271, 567.33372616, 314.1614329, 224.48692976 };
	const double intrinsic_n5[4] = { 573.52677509, 573.52154604, 314.03605057, 214.29659054 };*/

	const double intrinsic_n1[4] = { 571.15928878, 571.16352329, 311.07448762, 233.73421022 };
	const double intrinsic_n2[4] = { 572.19350868, 572.19147096, 311.08874623, 228.22643626 };
	const double intrinsic_n3[4] = { 569.22935734, 569.22937915, 308.72611105, 225.53660806 };
	const double intrinsic_n4[4] = { 567.62010676, 567.62984091, 311.41553264, 225.10048896 };
	const double intrinsic_n5[4] = { 572.97665944, 572.96466485, 310.95680422, 215.36230807 };

	std::vector<std::string> boundingBox12Path_h;
	std::vector<std::string> boundingBox12Path_v;
	std::vector<std::string> object_1_depthFileNames;
	std::vector<std::string> object_2_depthFileNames;

	std::size_t index = 0;
	int count = 0;

	std::string nameBoundingBoxFile = "";
	std::string nameDepthImageFile = "";
	std::string toEraseTXT = ".txt";
	std::string toErasePNG = ".png";

	std::cout << "cutPointCloudOfDetectObjects::Process data: Read data" << std::endl;

	cv::glob(object12PathInputBoundingBox_h, boundingBox12Path_h);
	cv::glob(object12PathInputBoundingBox_v, boundingBox12Path_v);
	cv::glob(object_1_depthPath, object_1_depthFileNames);
	cv::glob(object_2_depthPath, object_2_depthFileNames);

	std::sort(boundingBox12Path_h.begin(), boundingBox12Path_h.end(), naturalSorting);
	std::sort(boundingBox12Path_v.begin(), boundingBox12Path_v.end(), naturalSorting);

	std::sort(object_1_depthFileNames.begin(), object_1_depthFileNames.end(), naturalSorting);
	std::sort(object_2_depthFileNames.begin(), object_2_depthFileNames.end(), naturalSorting);

	std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " << std::endl;
	for (auto const& f : boundingBox12Path_h)
	{
		nameBoundingBoxFile = boundingBox12Path_h[index];
		eraseSubStrings(nameBoundingBoxFile, toEraseTXT);

		std::string nameFile = nameBoundingBoxFile.substr(nameBoundingBoxFile.find("\\") + 1);

		/* Read bounding box coordinate*/
		std::ifstream inputfile(nameBoundingBoxFile + ".txt");
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

		/* Check merge is horizontal or vertical*/
		nameBoundingBoxFile = boundingBox12Path_h[index];
		eraseSubStrings(nameBoundingBoxFile, toEraseTXT);
		std::string nameDepthImgCropped = nameBoundingBoxFile.substr(nameBoundingBoxFile.find("\\") + 1).substr(0, nameBoundingBoxFile.substr(nameBoundingBoxFile.find("\\") + 1).find("1_merge_"));

		for (int r = 0; r < 2; r++)
		{
			boundingArr[r][1] *= 2560;
			boundingArr[r][2] *= 1024;
			boundingArr[r][3] *= 2560;
			boundingArr[r][4] *= 1024;

			boundingArr[r][1] = boundingArr[r][1] - boundingArr[r][3] / 2;
			boundingArr[r][2] = boundingArr[r][2] - boundingArr[r][4] / 2;
			boundingArr[r][3] = boundingArr[r][1] + boundingArr[r][3];
			boundingArr[r][4] = boundingArr[r][2] + boundingArr[r][4];

			boundingArr[r][1] = (boundingArr[r][1] / 2560 * 1280) + 0;
			boundingArr[r][2] = (boundingArr[r][2] / 1024 * 480);
			boundingArr[r][3] = (boundingArr[r][3] / 2560 * 1280) + 0;
			boundingArr[r][4] = (boundingArr[r][4] / 1024 * 480) + 0;

			if (boundingArr[r][1] > 640 || boundingArr[r][3] > 640)
			{
				boundingArr[r][1] -= 640;
				boundingArr[r][3] -= 640;
			}

			if (boundingArr[r][0] == 0)
			{
				if (nameDepthImgCropped.substr(0, 3) == "NP1")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n1);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP2")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n2);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP3")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n3);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP4")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n4);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP5")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n5);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_h.ply" << std::endl;
				}
			}

			else if (boundingArr[r][0] == 1)
			{
				if (nameDepthImgCropped.substr(0, 3) == "NP1")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n1);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP2")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n2);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP3")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n3);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP4")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n4);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP5")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n5);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_h.ply" << std::endl;
				}
			}
		}	
		index++;
	}

	index = 0;

	for (auto const& f : boundingBox12Path_v)
	{
		nameBoundingBoxFile = boundingBox12Path_v[index];
		eraseSubStrings(nameBoundingBoxFile, toEraseTXT);

		std::string nameFile = nameBoundingBoxFile.substr(nameBoundingBoxFile.find("\\") + 1);

		/* Read bounding box coordinate*/
		std::ifstream inputfile(nameBoundingBoxFile + ".txt");
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

		/* Check merge is horizontal or vertical*/
		nameBoundingBoxFile = boundingBox12Path_v[index];
		eraseSubStrings(nameBoundingBoxFile, toEraseTXT);
		std::string nameDepthImgCropped = nameBoundingBoxFile.substr(nameBoundingBoxFile.find("\\") + 1).substr(0, nameBoundingBoxFile.substr(nameBoundingBoxFile.find("\\") + 1).find("1_merge_"));

		for (int r = 0; r < 2; r++)
		{
			boundingArr[r][1] *= 1280;
			boundingArr[r][2] *= 2048;
			boundingArr[r][3] *= 1280;
			boundingArr[r][4] *= 2048;

			boundingArr[r][1] = boundingArr[r][1] - boundingArr[r][3] / 2;
			boundingArr[r][2] = boundingArr[r][2] - boundingArr[r][4] / 2;
			boundingArr[r][3] = boundingArr[r][1] + boundingArr[r][3];
			boundingArr[r][4] = boundingArr[r][2] + boundingArr[r][4];

			boundingArr[r][1] = (boundingArr[r][1] / 1280 * 640) + 0;
			boundingArr[r][2] = (boundingArr[r][2] / 2048 * 960);
			boundingArr[r][3] = (boundingArr[r][3] / 1280 * 640) + 0;
			boundingArr[r][4] = (boundingArr[r][4] / 2048 * 960) + 0;

			if (boundingArr[r][2] > 480 || boundingArr[r][4] > 480)
			{
				boundingArr[r][2] -= 480;
				boundingArr[r][4] -= 480;
			}

			if (boundingArr[r][0] == 0)
			{
				if (nameDepthImgCropped.substr(0, 3) == "NP1")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n1);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP2")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n2);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP3")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n3);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP4")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n4);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP5")
				{
					cv::Mat obj_1_img = cv::imread(object_1_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_1;
					obj_1_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_1);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_1, intrinsic_n5);
					pcl::io::savePLYFileBinary(shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "1_shape0_v.ply" << std::endl;
				}
			}

			else if (boundingArr[r][0] == 1)
			{
				if (nameDepthImgCropped.substr(0, 3) == "NP1")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n1);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP2")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n2);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP3")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n3);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP4")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n4);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply" << std::endl;
				}
				else if (nameDepthImgCropped.substr(0, 3) == "NP5")
				{
					cv::Mat obj_2_img = cv::imread(object_2_depthFileNames[index], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
					cv::Mat croppedImg_obj_2;
					obj_2_img(cv::Rect(boundingArr[r][1], boundingArr[r][2], boundingArr[r][3] - boundingArr[r][1], boundingArr[r][4] - boundingArr[r][2])).copyTo(croppedImg_obj_2);

					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePointCloudFromDepthImage(croppedImg_obj_2, intrinsic_n5);
					pcl::io::savePLYFileBinary(shape1PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply", *cloud);
					std::cout << "cutPointCloudOfDetectObjects:: Process data: Process object 12: " + shape0PathOutput + "/" + nameDepthImgCropped + "2_shape1_v.ply" << std::endl;
				}
			}
		}
		index++;
	}
	std::cout << "cutPointCloudOfDetectObjects:: Finalizing" << std::endl;

}

void mergePointClouds()
{
	std::cout << "mergePointClouds:: Initialization" << std::endl;

	std::string mainFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/mergePointClouds";
	std::string inputFolder = mainFolder + "/Inputs";
	std::string outputFolder = mainFolder + "/Outputs";
	std::string debugFolder = mainFolder + "/Debugs";

	std::string pathDepth_1 = debugFolder + "/NP1_0_obj1_shape0_v.ply";
	std::string pathDepth_5 = debugFolder + "/NP5_0_obj1_shape0_v.ply";
	std::string pathDepth_192 = debugFolder + "/NP1_192_obj1_shape0_v.ply";

	const double intrinsic_n1[4] = { 571.15928878, 571.16352329, 311.07448762, 233.73421022 };
	const double intrinsic_n2[4] = { 572.19350868, 572.19147096, 311.08874623, 228.22643626 };
	const double intrinsic_n3[4] = { 569.22935734, 569.22937915, 308.72611105, 225.53660806 };
	const double intrinsic_n4[4] = { 567.62010676, 567.62984091, 311.41553264, 225.10048896 };
	const double intrinsic_n5[4] = { 572.97665944, 572.96466485, 310.95680422, 215.36230807 };

	/*cv::Mat depth_1 = cv::imread(pathDepth_1, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
	cv::Mat depth_5 = cv::imread(pathDepth_5, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 = createPointCloud(depth_1, intrinsic_n1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_5 = createPointCloud(depth_5, intrinsic_n5);*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_5(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_192(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPLYFile(pathDepth_1, *cloud_1);
	pcl::io::loadPLYFile(pathDepth_5, *cloud_5);
	pcl::io::loadPLYFile(pathDepth_192, *cloud_192);

	Eigen::Matrix4f NP1_NP5;
	NP1_NP5 << 0.99875184, 0.04598368, -0.0195003, -0.02709601,
		0.02167112, -0.04719246, 0.99865071, -0.83099701,
		0.04500137, -0.99782683, -0.04813008, 0.71722502,
		0., 0., 0., 1.;

	Eigen::Matrix4f N1_NP5;
	N1_NP5 << 0.99889519, -0.04588697, -0.01013818, -0.02475486,
		0.01385252, 0.08136559, 0.99658805, -0.86453359,
		-0.0449055, -0.99562745, 0.08191135, 0.62353965,
		0., 0., 0., 1.;
	Eigen::Matrix4f NP5_NP5 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f N5_NP5;
	N5_NP5 << 9.99918519e-01, -1.36529349e-04, 1.27646766e-02, -6.19782580e-02,
		-1.25058717e-04, 9.99790049e-01, 2.04900774e-02, 6.52560720e-02,
		-1.27647941e-02, -2.04900042e-02, 9.99708567e-01, 2.09527401e-02,
		0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;

	Eigen::Matrix4f H_table_NP5_1;
	H_table_NP5_1 << -0.10236438, -0.99210449, -0.07245836, 0.02107357,
		-0.99472137, 0.10156727, 0.01461105, 0.03693756,
		-0.00713629, 0.07357153, -0.99726441, 0.97146488,
		0., 0., 0., 1.;

	Eigen::Matrix4f H_table_NP5_192;
	H_table_NP5_192 << -1.06399879e-01, 9.91572358e-01, 7.39143091e-02, -1.29524560e-02,
		9.94297810e-01, 1.06636295e-01, 7.51741119e-04, -4.05050915e-02,
		-7.13654233e-03, 7.35728209e-02, -9.97264313e-01, 9.71464883e-01,
		0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
	
	pcl::transformPointCloud(*cloud_1, *cloud_1, N1_NP5.inverse());
	pcl::transformPointCloud(*cloud_1, *cloud_1, H_table_NP5_1);

	pcl::transformPointCloud(*cloud_192, *cloud_192, N1_NP5.inverse());
	pcl::transformPointCloud(*cloud_192, *cloud_192, H_table_NP5_192);

	pcl::transformPointCloud(*cloud_5, *cloud_5, N5_NP5.inverse());
	pcl::transformPointCloud(*cloud_5, *cloud_5, H_table_NP5_1);

	/*transformPointCloudToOriginal(cloud_1, cloud_1);
	transformPointCloudToOriginal(cloud_5, cloud_5);
	transformPointCloudToOriginal(cloud_192, cloud_192);*/
	
	pcl::io::savePLYFile(outputFolder + "/cloud_1.ply", *cloud_1);
	pcl::io::savePLYFile(outputFolder + "/cloud_5.ply", *cloud_5);
	pcl::io::savePLYFile(outputFolder + "/cloud_192.ply", *cloud_192);

	*finalCloud = *cloud_1 + *cloud_5 + *cloud_192;
	transformPointCloudToOriginal(finalCloud, finalCloud);

	pcl::io::savePLYFile(outputFolder + "/cloud15192.ply", *finalCloud);
	
	std::cout << "mergePointClouds:: Finalizing" << std::endl;

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
	/*cutPointCloudOfDetectObjects();*/
	mergePointClouds();
	/*Finalizing*/
	std::cout << "mainFunction:: Finalizing" << std::endl;
}