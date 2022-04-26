#include "test_ICP.h"

void process() {

    Eigen::Matrix4f N1toNP5;
    N1toNP5 << 0.99928155, -0.03780454, -0.00268218, -0.02972586,
        0.00526974, 0.06851348, 0.99763627, -0.85452296,
        -0.03753142, -0.99693366, 0.06866347, 0.63487774,
        0., 0., 0., 1.;
    Eigen::Matrix4f N2toNP5;
    N2toNP5 << 0.99874839, -0.04903673, 0.00985108, -0.03305605,
        0.0169128, 0.51646127, 0.85614351, -0.76596183,
        -0.04707017, -0.85490535, 0.51664421, 0.29782426,
        0., 0., 0., 1.;
    Eigen::Matrix4f N3toNP5;
    N3toNP5 << 9.99092384e-01, -2.60667955e-02, 3.36887372e-02, -7.57782945e-02,
        9.17920253e-04, 8.03883670e-01, 5.94785846e-01, -5.22380423e-01,
        -4.25859867e-02, -5.94215085e-01, 8.03177979e-01, 9.04279362e-02,
        0., 0., 0., 1.;
    Eigen::Matrix4f N4toNP5;
    N4toNP5 << 0.99931699, -0.01651251, 0.03305894, -0.05964673,
        0.00486274, 0.945592, 0.32531851, -0.22312939,
        -0.0366321, -0.32493555, 0.94502644, 0.03106564,
        0., 0., 0., 1.;

    Eigen::Matrix4f N5toNP5;
    N5toNP5 << 9.99894391e-01, -3.38543082e-05, 1.45329238e-02, -6.21285000e-02,
        -2.04278424e-04, 9.99865753e-01, 1.63839443e-02, 6.50627640e-02,
        -1.45315274e-02, -1.63851828e-02, 9.99760151e-01, 2.16908848e-02,
        0., 0., 0., 1.;
	
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_5(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> merge_cloud_1;
    pcl::PointCloud<pcl::PointXYZ> merge_cloud_2;
    pcl::PointCloud<pcl::PointXYZ> merge_cloud_3;
    pcl::PointCloud<pcl::PointXYZ> merge_cloud_4;
    pcl::PointCloud<pcl::PointXYZ> merge_cloud_5;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud4(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud5(new pcl::PointCloud<pcl::PointXYZ>());

    std::string np1_path = "D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP1_0.ply";
    std::string np2_path = "D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP2_0.ply";
    std::string np3_path = "D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP3_0.ply";
    std::string np4_path = "D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP4_0.ply";
    std::string np5_path = "D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP5_0.ply";

    pcl::io::loadPLYFile<pcl::PointXYZ>(np1_path, *cloud_1);
    pcl::io::loadPLYFile<pcl::PointXYZ>(np2_path, *cloud_2);
    pcl::io::loadPLYFile<pcl::PointXYZ>(np3_path, *cloud_3);
    pcl::io::loadPLYFile<pcl::PointXYZ>(np4_path, *cloud_4);
    pcl::io::loadPLYFile<pcl::PointXYZ>(np5_path, *cloud_5);

    pcl::transformPointCloud(*cloud_1, *transformed_cloud1, N1toNP5);
    pcl::transformPointCloud(*cloud_5, *transformed_cloud5, N5toNP5);
    
    pcl::io::savePLYFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/test_ICP_NP1.ply", *transformed_cloud1 + *transformed_cloud5);

}