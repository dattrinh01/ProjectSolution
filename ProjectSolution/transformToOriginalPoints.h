#include <iostream>
#include <math.h>
#include <string>
#include <string.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifndef _convert_matrixXd_to_pcl
#define _convert_matrixXd_to_pcl
pcl::PointCloud<pcl::PointXYZ> convertEigenMatrixXdToPCLCloud
(
	const Eigen::MatrixXd& inputMatrix
);
#endif

#ifndef _convert_pcl_to_matrixXd
#define _convert_pcl_to_matrixXd
Eigen::MatrixXd convertPCLCloudToEigenMatrixXd
(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud
);
#endif

#ifndef _transform_vertical_point_to_point
#define _transform_vertical_point_to_point
Eigen::Matrix4f transformVerticesFromPointToPoint
(
	const Eigen::MatrixXd& targetVertices,
	const Eigen::Vector3d fromPoint,
	const Eigen::Vector3d toPoint,
	Eigen::MatrixXd& outPoints
);
#endif

#ifndef _transform_point_using_transform_mat
#define _transform_point_using_transform_mat
Eigen::MatrixXd transformPointsWithTransformMatrix(
	const Eigen::MatrixXd& inputVertices,
	const Eigen::Matrix4f& transformMatrix
);
#endif

#ifndef _calculate_centroid_point_cloud
#define _calculate_centroid_point_cloud
void findCentroidOfPointCloud(std::string pathPointCloud);
#endif // !_apply_point_cloud_to_matrix
