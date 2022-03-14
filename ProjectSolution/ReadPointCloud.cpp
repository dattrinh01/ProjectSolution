#include "ReadPointCloud.h"

void read_pcd(std::string pointCloudPath) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(pointCloudPath, *cloud) != 0)
	{
	}
	pcl::visualization::CloudViewer viewer("Point cloud");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped()) {}
}