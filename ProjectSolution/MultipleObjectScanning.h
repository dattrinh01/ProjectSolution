#ifndef _MULTIPLE_OBJECT_SCANNING
#define _MULTIPLE_OBJECT_SCANNING

#include <iostream>
#include <vector>
#include <string.h>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>

/*-----------SUPPORT FUNCTIONS-----------*/
bool naturalSorting(const std::string& a, const std::string& b);
void eraseSubStrings(std::string& mainString, std::string& toErase);
pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloud(cv::Mat depth_img, const double depth_intrinsic[4]);
bool checkSubString(std::string mainString, std::string checkString);
void extractBoundingBoxFromMaskImage(cv::Mat mask_img, double& bbX, double& bbY, double& bbWidth, double& bbHeight);
/*-----------MAIN PROCESSING FUNCTIONS-----------*/
void datasetGeneration();
void detectMultipleObjects();
void cutPointCloudOfDetectObjects();
void mergePointClouds();
void meshPointClouds();
void evaluationPointClouds();
/*-----------MAIN FUNCTIONS-----------*/
void mainFunction();

#endif // !_MULTIPLE_OBJECT_SCANNING