#include "transformToOriginalPoints.h"

pcl::PointCloud<pcl::PointXYZ> convertEigenMatrixXdToPCLCloud(const Eigen::MatrixXd& inputMatrix)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
	outputCloud->points.resize(inputMatrix.rows());
	for (unsigned int i = 0; i < outputCloud->size(); i++)
	{
		pcl::PointXYZ point;
		point.x = inputMatrix(i, 0);
		point.y = inputMatrix(i, 1);
		point.z = inputMatrix(i, 2);

		outputCloud->points[i] = point;
	}
	return *outputCloud;
}

Eigen::MatrixXd convertPCLCloudToEigenMatrixXd(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
	Eigen::MatrixXd outputBuffer(inputCloud->points.size(), 3);
	for (unsigned int i = 0; i < outputBuffer.rows(); i++)
	{
		Eigen::RowVector3d point = Eigen::RowVector3d(inputCloud->points[i].x, inputCloud->points[i].y, inputCloud->points[i].z);
		outputBuffer.row(i) = point;
	}
	return outputBuffer;
}

Eigen::Matrix4f transformVerticesFromPointToPoint
(
	const Eigen::MatrixXd& targetVertices, 
	const Eigen::Vector3d fromPoint, 
	const Eigen::Vector3d toPoint, 
	Eigen::MatrixXd& outPoints
)
{
	Eigen::MatrixXd outputBuffer;
	Eigen::Vector3d diffVector = (toPoint - fromPoint);
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0, 3) = diffVector.x();
	transform(1, 3) = diffVector.y();
	transform(2, 3) = diffVector.z();

	outputBuffer = transformPointsWithTransformMatrix(targetVertices, transform);
	outPoints = outputBuffer;
	return transform;
}

Eigen::MatrixXd transformPointsWithTransformMatrix
(
	const Eigen::MatrixXd& inputVertices,
	const Eigen::Matrix4f& transformMatrix
)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
	Eigen::MatrixXd outputMatrix;
	*sourceCloud = convertEigenMatrixXdToPCLCloud(inputVertices);
	pcl::transformPointCloud(*sourceCloud, *transformedCloud, transformMatrix);
	outputMatrix = convertPCLCloudToEigenMatrixXd(transformedCloud);
	return outputMatrix;
}

void findCentroidOfPointCloud(std::string pathPointCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr PCLCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPLYFile(pathPointCloud, *PCLCloud);

	Eigen::MatrixXd cloudMatrix = convertPCLCloudToEigenMatrixXd(PCLCloud);
	std::cout << cloudMatrix.colwise().mean();
}