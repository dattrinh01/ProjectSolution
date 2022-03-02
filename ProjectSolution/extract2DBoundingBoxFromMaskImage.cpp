#include "extract2DBoundingBoxFromMaskImage.h"

void get_bounding_box_from_mask_image(cv::Mat mask_img, int& xMin, int& yMin, int& xMax, int& yMax) {

	cv::Mat mSource_Gray, mThreshold;
	cv::cvtColor(mask_img, mSource_Gray, cv::COLOR_BGR2GRAY);
	cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);
	cv::Mat Points;
	findNonZero(mThreshold, Points);
	cv::Rect Min_Rect = boundingRect(Points);
	xMin = Min_Rect.tl().x; 	xMax = Min_Rect.br().x; 	yMin = Min_Rect.tl().y; 	yMax = Min_Rect.br().y;

	/*
	std::vector<std::string> fileNames;
	cv::glob("D:/DATA/Research/DrNhuResearch/test_data/masks/*.pbm", fileNames, false);

	std::size_t i = 0;
	for (auto const& f : fileNames) {
		cv::Mat mSource_Bgr, mSource_Gray, mThreshold;
		mSource_Bgr = cv::imread(fileNames[i], 1);
		cv::cvtColor(mSource_Bgr, mSource_Gray, cv::COLOR_BGR2GRAY);
		cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);

		cv::Mat Points;
		findNonZero(mThreshold, Points);
		cv::Rect Min_Rect = boundingRect(Points);

		xMin = Min_Rect.tl().x;
		xMax = Min_Rect.br().x;
		yMin = Min_Rect.tl().y;
		yMax = Min_Rect.br().y;

		//get name for saving image
		/*std::string toErase = "D:/DATA/Research/DrNhuResearch/RGBDData/rgbd/masks//";
		std::size_t pos = fileNames[i].find(toErase);
		if (pos != std::string::npos)
		{
			fileNames[i].erase(pos, toErase.length());
		}
		std::cout << fileNames[i] << std::endl;
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));
		fileNames[i].erase(std::prev(fileNames[i].end()));*/

		

		// save rgb images with draw bounding box
		/*fileNames[i] += ".png";
		cv::rectangle(rgb, Min_Rect.tl(), Min_Rect.br(), cv::Scalar(0, 255, 0), 2);

		cv::imwrite("rs/"+ fileNames[i], rgb);*/
		// save bounding box coordinates into txt file
		/*fileNames[i] += ".txt";
		std::string output = std::to_string(Min_Rect.tl().x) + " " + std::to_string(Min_Rect.br().x) + " " +
			std::to_string(Min_Rect.tl().y) + " " + std::to_string(Min_Rect.br().y);
		std::ofstream out("bb/" + fileNames[i]);
		out << output;
		out.close();
		i++;
		std::cout << i << "/600" << std::endl;*/

	//}
}