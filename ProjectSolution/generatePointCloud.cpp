#include "generatePointCloud.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(cv::Mat depth_img, cv::Mat mask_img, const double depth_intrinsic[4]) {

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