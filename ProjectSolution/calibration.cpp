#include "calibration.h"

void calib(std::vector<cv::String>fileNames, int patternSizeX, int patternSizeY, cv::Matx33f& cameraMatrix, cv::Vec<float, 5>& distortionMatrix) {

	cv::Size patternSize = { patternSizeX - 1, patternSizeY - 1 };
	std::vector<std::vector<cv::Point2f>> q(fileNames.size());
	std::vector<std::vector<cv::Point3f>> Q;
	std::vector<cv::Point3f> worldCoord;


	int chessboard[2] = { patternSizeX, patternSizeY };

	for (int i = 1; i < chessboard[1]; i++) {
		for (int j = 1; j < chessboard[0]; j++) {
			worldCoord.push_back(cv::Point3f(j, i, 0));
		}
	}

	std::vector<cv::Point2f> imgPoint;
	std::size_t i = 0;

	for (auto const& f : fileNames) {
		std::cout << "Read " + std::string(f) << std::endl;

		cv::Mat gray;
		cv::Mat img = cv::imread(fileNames[i]);
		if (img.empty()) {
			std::cout << "Failed to read this image" << std::endl;
		}
		else {
			std::cout << "Load image is ok" << std::endl;
		}

		cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
		int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
		bool checkPatterFound = cv::findChessboardCorners(gray, patternSize, q[i], flags);
		
		if (checkPatterFound) {
			cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
			Q.push_back(worldCoord);
		}
		
		cv::drawChessboardCorners(img, patternSize, q[i], checkPatterFound);
		i++;
	}

	std::vector<cv::Mat> R, T;
	int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
	float error = cv::calibrateCamera(Q, q, cv::Size(1280, 720), cameraMatrix, distortionMatrix, R, T, flags);

	std::cout << "Camera Matrix:" << std::endl;
	std::cout << cameraMatrix << std::endl;
	std::cout << "Distorsion Matrix:" << std::endl;
	std::cout << distortionMatrix << std::endl;
	std::cout << "Error calibration: " << error << std::endl;

	cv::Mat mapX, mapY;
	cv::initUndistortRectifyMap(cameraMatrix, distortionMatrix, cv::Matx33f::eye(), cameraMatrix, cv::Size(1280, 720), CV_32FC1, mapX, mapY);
	for (auto const& f : fileNames) {
		cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
		cv::Mat imgUndistortion;
		cv::remap(img, imgUndistortion, mapX, mapY, cv::INTER_LINEAR);
		cv::imshow("Undistortion images", imgUndistortion);
		cv::waitKey(0);
	}
}
/*
// Camera calibration main // 

#include "calibration.h"

int main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;

	std::vector<cv::String> fileNames;
	cv::glob("D:/DATA/Research/DrNhuResearch/RGBDData/CLUBS_data/CLUBS_calibration_data/calibration_data/realsense_d435/temp/*.png", fileNames, false);

	cv::Mat mapX, mapY;
	cv::Vec<float, 5> distortionMatrix(0, 0, 0, 0, 0);
	cv::Matx33f cameraMatrix(cv::Matx33f::eye());

	int patternSizeX = 8;
	int patternSizeY = 7;

	int width = 1280;
	int height = 720;

	calib(fileNames, patternSizeX, patternSizeY, cameraMatrix, distortionMatrix);
	cv::initUndistortRectifyMap(cameraMatrix, distortionMatrix, cv::Matx33f::eye(), cameraMatrix, cv::Size(width, height), CV_32FC1, mapX, mapY);
	cv::Mat imgUndistortion;

	int cameraNumber = 0;
	if (argc > 1) {
		cameraNumber = atoi(argv[1]);
	}
	cv::VideoCapture camera;
	camera.open(cameraNumber);

	if (!camera.isOpened()) {
		std::cout << "Can't open your camera" << std::endl;
	}

	camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);

	while (true) {
		cv::Mat cameraFrame;
		camera >> cameraFrame;

		if (cameraFrame.empty()) {
			std::cout << "Can't access your camera" << std::endl;
			exit(1);
		}
		cv::remap(cameraFrame, imgUndistortion, mapX, mapY, cv::INTER_LINEAR);
		cv::imshow("Undistortion image", imgUndistortion);
		cv::waitKey(0);
	}
	return 0;
}
// Camera calibration main // 
*/