#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

using namespace std;

int main(int argc, char** argv)
{
	// �������
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// ��ȡPCD�ļ�
	if (pcl::io::loadPCDFile("D:\\data\\dragon\\dragon_1.pcd", *inputCloud) == -1)
	{
		cerr << "�޷���ȡ�ļ�" << endl;
		return -1;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Vector4f centroid;                    // ����
	Eigen::Matrix3f covarianceMatrix;           // Э�������

	// ������Ƶľ�ֵ��Э�������
	pcl::computeMeanAndCovarianceMatrix(*inputCloud, covarianceMatrix, centroid); //����Ϊ������꣨c0, c1, c2, 1��

	// ������
	cout << "���ƵĹ�һ��3x3Э�������Ϊ��\n" << covarianceMatrix << endl;
	cout << "��������Ϊ��" << endl
		<< "\t" << centroid(0)
		<< "\t" << centroid(1)
		<< "\t" << centroid(2)
		<< "\t" << centroid(3)
		<< endl;

	return 0;

}
