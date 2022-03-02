#include "extract2DBoundingBoxFromMaskImage.h"
#include "generatePointCloud.h"
#include "extractPointCloudInBoundingBox.h"

void extract_PointCloud_from_Bounding_Box() {
	std::string depth_path = "D:/DATA/Research/DrNhuResearch/test_data/depth_png/*.png";
	std::string mask_path = "D:/DATA/Research/DrNhuResearch/test_data/masks/*.pbm";
	cutPointCloud(depth_path, mask_path);
}

int main(int argc, char* argv[]) {
	extract_PointCloud_from_Bounding_Box();

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("D:/DATA/Research/DrNhuResearch/RGBDData/bigbird_db/BigBird_point_cloud/red_bull/clouds/NP4_0.pcd", *cloud) != 0)
	{
		return -1;
	}

	pcl::visualization::CloudViewer viewer("PC");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}*/
	
	return 0;
}