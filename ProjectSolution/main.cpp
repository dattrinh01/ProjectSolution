#include "extract2DBoundingBoxFromMaskImage.h"
#include "extractPointCloudInBoundingBox.h"
#include "generatePointCloud.h"

void test_extract_PointCloud_from_Bounding_Box() {
	std::string depth_path = "D:/DATA/Research/DrNhuResearch/test_data/depth_png/*.png";
	std::string mask_path = "D:/DATA/Research/DrNhuResearch/test_data/masks/*.pbm";
	cutPointCloud(depth_path, mask_path);
}

void test_bounding_box_mask_image() {
	std::vector<std::string> mask_fileNames, depth_fileNames;
	std::string mask_path = "D:/DATA/Research/DrNhuResearch/test_data/masks/*.pbm";
	std::string depth_path = "D:/DATA/Research/DrNhuResearch/test_data/depth_png/*.png";

	cv::glob(mask_path, mask_fileNames, false);
	cv::glob(depth_path, depth_fileNames, false);

	std::size_t i = 0;
	for (auto const& f : mask_fileNames) {

		cv::Mat mask_img = cv::imread(mask_fileNames[i]);
		cv::Mat depth_img = cv::imread(depth_fileNames[i], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

		cv::Mat mSource_Gray, mThreshold;
		cv::cvtColor(mask_img, mSource_Gray, cv::COLOR_BGR2GRAY);
		cv::threshold(mSource_Gray, mThreshold, 254, 255, cv::THRESH_BINARY_INV);
		cv::Mat Points;
		findNonZero(mThreshold, Points);
		cv::Rect Min_Rect = boundingRect(Points);

		int xMin = Min_Rect.tl().x;
		int xMax = Min_Rect.br().x;
		int yMin = Min_Rect.tl().y;
		int yMax = Min_Rect.br().y;

		//cv::rectangle(mask_img, cv::Point(xMin, yMin), cv::Point(xMax, yMax), cv::Scalar(0, 0, 255));
		cv::resize(mask_img, mask_img, cv::Size(depth_img.cols, depth_img.rows), cv::INTER_LINEAR);
		cv::imwrite("depth_bb/" + std::to_string(i) + ".png", mask_img);
		std::cout << i << std::endl;
		i++;
	}

}

int main(int argc, char* argv[]) {
	//test_extract_PointCloud_from_Bounding_Box();
	test_bounding_box_mask_image();
	
	
	return 0;
}