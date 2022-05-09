#include "transformToOriginalPoints.h"

pcl::PointCloud<pcl::PointXYZ> convertEigenMatrixXdToPCLCloud(const Eigen::MatrixXd& inputMatrix, int height, int width)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
	outputCloud->points.resize(inputMatrix.rows());
	/*outputCloud->height = inputMatrix.rows();
	outputCloud->width = 3;*/

	outputCloud->height = height;
	outputCloud->width = width;

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
		outputBuffer.row(i) = point / 1000.0;
	}
	return outputBuffer;
}

Eigen::Matrix4f transformVerticesFromPointToPoint(const Eigen::MatrixXd& targetVertices, 
	const Eigen::Vector3d fromPoint, const Eigen::Vector3d toPoint, Eigen::MatrixXd& outPoints)
{
	Eigen::MatrixXd outputBuffer;
	Eigen::Vector3d diffVector = (toPoint - fromPoint);
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 3) = diffVector.x(); transform(1, 3) = diffVector.y(); transform(2, 3) = diffVector.z();
	outputBuffer = transformPointsWithTransformMatrix(targetVertices, transform);
	outPoints = outputBuffer;
	return transform;
}

Eigen::MatrixXd transformPointsWithTransformMatrix(const Eigen::MatrixXd inputVertices, const Eigen::Matrix4f transformMatrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
	Eigen::MatrixXd outputMatrix;

	*sourceCloud = convertEigenMatrixXdToPCLCloud(inputVertices, 4413 , 3);
	pcl::transformPointCloud(*sourceCloud, *transformedCloud, transformMatrix);
	outputMatrix = convertPCLCloudToEigenMatrixXd(transformedCloud);
	return outputMatrix;
}

void transformPCToOrginalCoor(std::string pathPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	
	std::cout << "Load pcd point cloud from " << pathPointCloud << std::endl;
	pcl::io::loadPLYFile(pathPointCloud, *cloud);
	
	const Eigen::MatrixXd cloudMatrix = convertPCLCloudToEigenMatrixXd(cloud);

	std::cout << "Initial targetVertical --------------------" << std::endl;
	Eigen::MatrixXd targetVertical = cloudMatrix;
	std::cout << "Initial fromPoints --------------------" << std::endl;
	Eigen::Vector3d fromPoints = cloudMatrix.colwise().mean();
	std::cout << fromPoints << std::endl;
	std::cout << "Initial toPoints --------------------" << std::endl;
	Eigen::Vector3d toPoints(0.0, 0.0, 0.0);
	std::cout << "Initial outPoints --------------------" << std::endl;
	Eigen::MatrixXd outPoints;
	std::cout << "Calculate transformation matrix ---------------" << std::endl;
	Eigen::Matrix4f transformMat = transformVerticesFromPointToPoint(targetVertical, fromPoints, toPoints, outPoints);

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(new pcl::PointCloud<pcl::PointXYZ>());
	*transformCloud = convertEigenMatrixXdToPCLCloud(outPoints);

	pcl::io::savePLYFile("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP1_pc_trans.ply", *transformCloud);*/
}

