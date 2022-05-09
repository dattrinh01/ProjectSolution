#include "test_genPCMethod.h"

Eigen::MatrixXf readCSV(std::string file, int rows, int cols) {

    std::ifstream in(file);

    std::string line;

    int row = 0;
    int col = 0;

    Eigen::MatrixXf res = Eigen::MatrixXf(rows, cols);

    if (in.is_open()) {

        while (std::getline(in, line)) {

            char* ptr = (char*)line.c_str();
            int len = line.length();

            col = 0;

            char* start = ptr;
            for (int i = 0; i < len; i++) {

                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);

            row++;
        }

        in.close();
    }
    return res;
}

bool compareNat(const std::string& a, const std::string& b)
{
    if (a.empty())
        return true;
    if (b.empty())
        return false;
    if (std::isdigit(a[0]) && !std::isdigit(b[0]))
        return true;
    if (!std::isdigit(a[0]) && std::isdigit(b[0]))
        return false;
    if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
    {
        if (std::toupper(a[0]) == std::toupper(b[0]))
            return compareNat(a.substr(1), b.substr(1));
        return (std::toupper(a[0]) < std::toupper(b[0]));
    }

    /*Both strings begin with digit --> parse both numbers*/
    std::istringstream issa(a);
    std::istringstream issb(b);
    int ia, ib;
    issa >> ia;
    issb >> ib;
    if (ia != ib)
        return ia < ib;

    /*Numbers are the same --> remove numbers and recurse*/
    std::string anew, bnew;
    std::getline(issa, anew);
    std::getline(issb, bnew);
    return (compareNat(anew, bnew));
}

void choose_matrix_each_view(std::string data_path) {

    std::vector<std::string> rgb_fileNames;
    cv::glob(data_path, rgb_fileNames);
    std::sort(rgb_fileNames.begin(), rgb_fileNames.end(), compareNat);

    

    unsigned int i = 0;
    for (auto const& f : rgb_fileNames) {
        unsigned int j = 0;
        while (j <= 3) {

            std::cout << rgb_fileNames[i] << std::endl;

            j++;
            i++;
        }
        std::cout << "--------------------------------------------------------" << std::endl;
        j = 0;

    }

}


void test_process_frame(std::string data_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud4(new pcl::PointCloud<pcl::PointXYZ>());

    const double intrinsic_n1[4] = { 570.31691719, 570.31988743, 314.9278689 , 228.59060364 };
    const double intrinsic_n2[4] = { 572.31327882, 572.31155976, 314.34439596, 228.35836425 };
	const double intrinsic_n3[4] = { 568.84098602, 568.84158184, 313.36329231, 224.84618475 };
    const double intrinsic_n4[4] = { 567.32716271, 567.33372616, 314.1614329 , 224.48692976 };
    const double intrinsic_n5[4] = { 573.52677509, 573.52154604, 314.03605057, 214.29659054 };

    Eigen::Matrix4f NP1toNP5;
    NP1toNP5 << 0.99843731, 0.05475579, -0.01116875, -0.03335538,
        0.01421443, -0.05555295, 0.99835456, -0.82327339,
        0.05404523, -0.9969532, -0.05624446, 0.72417926,
        0., 0., 0., 1.;
    Eigen::Matrix4f NP2toNP5;
    NP2toNP5 << 0.99874538, 0.01566433, -0.04756353, 0.01239749,
        0.03735488, 0.39952589, 0.91596052, -0.77269939,
        0.03335078, -0.91658807, 0.3984395, 0.38541563,
        0., 0., 0., 1.;
    Eigen::Matrix4f NP3toNP5;
    NP3toNP5 << 0.99908355, 0.0032791, -0.04267682, -0.01360154,
        0.02657465, 0.7340928, 0.67852895, -0.56790004,
        0.03355371, -0.67904124, 0.7333329, 0.13025396,
        0., 0., 0., 1.;
    Eigen::Matrix4f NP4toNP5;
    NP4toNP5 << 0.99917161, -0.01583408, -0.03748825, 0.00281982,
        0.02972207, 0.91317723, 0.40647748, -0.27856183,
        0.02779722, -0.40725499, 0.91289139, 0.03491665,
        0., 0., 0., 1.;
    Eigen::Matrix4f NP5toNP5 = Eigen::Matrix4f::Identity();
    /*-----------------------------------------------------------*/

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

    std::string rgb_path = data_path + "/rgb_test_one_frame/*.jpg";
    std::string mask_path = data_path + "/mask_test_one_frame/*.pbm";
    std::string depth_path = data_path + "/depth_test_one_frame/*.png";
    std::vector<std::string> rgb_fileNames, mask_fileNames, depth_fileNames;

    cv::glob(rgb_path, rgb_fileNames, false);
    cv::glob(mask_path, mask_fileNames, false);
    cv::glob(depth_path, depth_fileNames, false);

    int index_fileNames = 0;

    std::string mergeCloudPath = "D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/allPointCloud.ply";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_5(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto const& f : depth_fileNames)
    {
        cv::Mat croppedImg;
        cv::Mat rgb_image = cv::imread(rgb_fileNames[index_fileNames]);
        cv::Mat mask_image = cv::imread(mask_fileNames[index_fileNames]);
        cv::Mat depth_image = cv::imread(depth_fileNames[index_fileNames], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

        double bbX, bbY, bbWidth, bbHeight;
        get_bounding_box_from_mask_image(mask_image, depth_image, bbX, bbY, bbWidth, bbHeight);
        depth_image(cv::Rect(bbX, bbY, bbWidth, bbHeight)).copyTo(croppedImg);


        

        if (depth_fileNames[index_fileNames][79] == '5')
        {
            cloudC = generatePointCloud(croppedImg, intrinsic_n5);
            /*pcl::transformPointCloud(*cloud_5, *cloud_5, NP5toNP5);*/
            pcl::io::savePCDFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP5_pc.pcd", *cloudC);
            /*pcl::io::savePLYFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP5_pc.ply", *cloudC);*/
        }

        if (depth_fileNames[index_fileNames][79] == '4')
        {
            cloud_1 = generatePointCloud(croppedImg, intrinsic_n4);
            /*pcl::transformPointCloud(*cloud_1, *cloud_1, N4toNP5);*/
            pcl::io::savePCDFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP4_pc.pcd", *cloud_1);
        }

        if (depth_fileNames[index_fileNames][79] == '1')
        {
            cloudC = generatePointCloud(croppedImg, intrinsic_n1);
            /*pcl::transformPointCloud(*cloud_5, *cloud_5, NP5toNP5);*/

            pcl::io::savePCDFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP1_pc.pcd", *cloudC);
        }

        if (depth_fileNames[index_fileNames][79] == '2')
        {
            cloudC = generatePointCloud(croppedImg, intrinsic_n2);
            /*pcl::transformPointCloud(*cloud_5, *cloud_5, NP5toNP5);*/

            pcl::io::savePCDFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP2_pc.pcd", *cloudC);
        }

        if (depth_fileNames[index_fileNames][79] == '3')
        {
            cloudC = generatePointCloud(croppedImg, intrinsic_n3);
            /*pcl::transformPointCloud(*cloud_5, *cloud_5, NP5toNP5);*/

            pcl::io::savePCDFileBinary("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/NP3_pc.pcd", *cloudC);
        }
        index_fileNames++;
    }

    /**cloudC += *cloud_1;
    pcl::io::savePLYFile("D:/DATA/Research/DrNhu/demoData/red_bull/one_frame_data/pointCloud/merge.ply", *cloudC);*/
}
