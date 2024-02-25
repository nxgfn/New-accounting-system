#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h> // ʹ�ü����ֵ��ͷ�ļ�
#include <pcl/common/common.h> // ʹ�ü��������С�������ͷ�ļ�

// �����������ľ�ֵ�ͱ�׼��
void calculateMeanStd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& mean, pcl::PointXYZ& standardDeviation)
{
    if (cloud->empty()) {
        std::cerr << "�����������Ϊ�ա�" << std::endl;
        return;
    }

    // �����ֵ
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    mean.x = centroid[0];
    mean.y = centroid[1];
    mean.z = centroid[2];

    // �����׼��
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    standardDeviation.x = (maxPt[0] - minPt[0]) / 2.0f;
    standardDeviation.y = (maxPt[1] - minPt[1]) / 2.0f;
    standardDeviation.z = (maxPt[2] - minPt[2]) / 2.0f;
}

int main(int argc, char** argv)
{
    // ���ص�������
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("D://data//dragon//dragon_1.pcd", *cloud) == -1)
    {
        std::cerr << "�����޷���ȡ�ļ���" << std::endl;
        return -1;
    }

    // �����ֵ�ͱ�׼��
    pcl::PointXYZ mean, standardDeviation;
    calculateMeanStd(cloud, mean, standardDeviation);

    // ��ʾ���
    std::cout << "��ֵ��x=" << mean.x << " y=" << mean.y << " z=" << mean.z << std::endl;
    std::cout << "��׼�x=" << standardDeviation.x << " y=" << standardDeviation.y << " z=" << standardDeviation.z << std::endl;

    return 0;
}
