// pcl_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//#include <iostream>
//
//int main()
//{
//    std::cout << "Hello World!\n"; 
//}


// 点的类型的头文件
#include <pcl/point_types.h>
// 点云文件 IO（pcd 文件和 ply 文件）
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//kd 树
#include <pcl/kdtree/kdtree_flann.h>
// 特征提取
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
// 重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
// 可视化
#include <pcl/visualization/pcl_visualizer.h>
// 多线程
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

int main(int argc, char** argv)
{
	//1.loadPCDFile读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/test.pcd", *cloud1) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "1.loadPCDFile方式使用指针读取点个数: " << cloud1->points.size() << endl;

	return 0;
}



// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
