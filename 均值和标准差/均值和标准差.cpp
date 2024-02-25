#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h> // 使用计算均值的头文件
#include <pcl/common/common.h> // 使用计算点云最小、最大点的头文件

// 计算点云坐标的均值和标准差
void calculateMeanStd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointXYZ& mean, pcl::PointXYZ& standardDeviation)
{
    if (cloud->empty()) {
        std::cerr << "错误：输入点云为空。" << std::endl;
        return;
    }

    // 计算均值
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    mean.x = centroid[0];
    mean.y = centroid[1];
    mean.z = centroid[2];

    // 计算标准差
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    standardDeviation.x = (maxPt[0] - minPt[0]) / 2.0f;
    standardDeviation.y = (maxPt[1] - minPt[1]) / 2.0f;
    standardDeviation.z = (maxPt[2] - minPt[2]) / 2.0f;
}

int main(int argc, char** argv)
{
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("D://data//dragon//dragon_1.pcd", *cloud) == -1)
    {
        std::cerr << "错误：无法读取文件。" << std::endl;
        return -1;
    }

    // 计算均值和标准差
    pcl::PointXYZ mean, standardDeviation;
    calculateMeanStd(cloud, mean, standardDeviation);

    // 显示结果
    std::cout << "均值：x=" << mean.x << " y=" << mean.y << " z=" << mean.z << std::endl;
    std::cout << "标准差：x=" << standardDeviation.x << " y=" << standardDeviation.y << " z=" << standardDeviation.z << std::endl;

    return 0;
}
