#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

using namespace std;

int main(int argc, char** argv)
{
	// 导入点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 读取PCD文件
	if (pcl::io::loadPCDFile("D:\\data\\dragon\\dragon_1.pcd", *inputCloud) == -1)
	{
		cerr << "无法读取文件" << endl;
		return -1;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Vector4f centroid;                    // 质心
	Eigen::Matrix3f covarianceMatrix;           // 协方差矩阵

	// 计算点云的均值和协方差矩阵
	pcl::computeMeanAndCovarianceMatrix(*inputCloud, covarianceMatrix, centroid); //质心为齐次坐标（c0, c1, c2, 1）

	// 输出结果
	cout << "点云的归一化3x3协方差矩阵为：\n" << covarianceMatrix << endl;
	cout << "点云质心为：" << endl
		<< "\t" << centroid(0)
		<< "\t" << centroid(1)
		<< "\t" << centroid(2)
		<< "\t" << centroid(3)
		<< endl;

	return 0;

}
