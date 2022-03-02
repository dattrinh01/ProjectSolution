#include "generatePointCloud.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr generate_point_cloud(cv::Mat depth_img, const double rgb_intrinsic[4]) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	const double fx = rgb_intrinsic[0];
	const double fy = rgb_intrinsic[1];
	const double cx = rgb_intrinsic[2];
	const double cy = rgb_intrinsic[3];

	for (int i = 0; i < depth_img.rows; i++) {
		for (int j = 0; j < depth_img.cols; j++) {

			float depth_val = depth_img.at<uint16_t>(i, j);

			if (depth_val == 0.0f) {
				continue;
			}

			pcl::PointXYZ p;
			p.z = double(depth_val) / 5000;
			p.x = (j - cx) * p.z / fx / 5000;
			p.y = (i - cy) * p.z / fy / 5000;
			cloud->points.push_back(p);
		}
	}
	return cloud;
}
