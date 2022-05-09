#include "MultipleObjectScanning.h"
/*-----------SUPPORT FUNCTIONS-----------*/
/*-----------MAIN PROCESSING FUNCTIONS-----------*/
/*-----------MAIN FUNCTIONS-----------*/

void datasetGeneration()
{
	/*Initialization*/
	std::cout << "datasetGeneration:: Initialization" << std::endl;
	const std::string mainFolder = "D:/DATA/Research/DrNhu/ProjectSolutionFolder/datasetGeneration";
	const std::string inputFolder = mainFolder + "/Inputs";
	const std::string outputFolder = mainFolder + "/Outputs";
	const std::string debugFolder = mainFolder + "/Debugs";


	/*Read data*/
	std::cout << "datasetGeneration:: Read data" << std::endl;

	/*Process data*/
	std::cout << "datasetGeneration:: Process data" << std::endl;

	/*Finalizing*/
	std::cout << "datasetGeneration:: Finalizing" << std::endl;
}

void detectMultipleObjects()
{
}

void cutPointCloudOfDetectObjects()
{
}

void mergePointClouds()
{
}

void meshPointClouds()
{
}

void evaluationPointClouds()
{
}

void mainFunction()
{
	/*Initialization*/
	std::cout << "mainFunction:: Initialization" << std::endl;

	/*Input data*/
	std::cout << "mainFunction:: Input data" << std::endl;

	/*Process data*/
	std::cout << "mainFunction:: Process data" << std::endl;
	datasetGeneration();
	/*Finalizing*/
	std::cout << "mainFunction:: Finalizing" << std::endl;
}
