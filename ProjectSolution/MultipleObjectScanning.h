#ifndef _MULTIPLE_OBJECT_SCANNING
#define _MULTIPLE_OBJECT_SCANNING

#include <iostream>
#include <vector>
#include <string.h>
#include <string>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

/*-----------SUPPORT FUNCTIONS-----------*/

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
