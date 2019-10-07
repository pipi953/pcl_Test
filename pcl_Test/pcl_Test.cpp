// pcl_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//#include <iostream>
//
//int main()
//{
//    std::cout << "Hello World!\n"; 
//}

/*
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

#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
	//1.loadPCDFile读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/bunny.pcd", *cloud1) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "1.loadPCDFile方式使用指针读取点个数: " << cloud1->points.size() << endl;

	pcl::PLYWriter writer;
	writer.write("tester.ply", *cloud1);

	pcl::visualization::CloudViewer viewer("pcd viewer");
	viewer.showCloud(cloud1);
	system("pause");


	return 0;
}
*/


#include <pcl/point_types.h>	//点类型相关定义
#include <pcl/io/pcd_io.h>	//文件输入输出
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include "data.h"


#include <iostream>
#include <fstream>
#include <strstream>
#include <vector>



////定义3D点的结构体
//struct Point3D
//{
//	float x;
//	float y;
//	float z;
//};


using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


bool readTxtFile(const string &fileName, const char tag, const PointCloudT::Ptr &pointCloud)
{
	cout << "reading file start..... " << endl;
	ifstream fin(fileName);
	string linestr;
	vector<PointT> myPoint;
	while (getline(fin, linestr))
	{
		vector<string> strvec;
		string s;
		stringstream ss(linestr);
		while (getline(ss, s, tag))
		{
			strvec.push_back(s);
		}
		if (strvec.size() < 3) {
			cout << "格式不支持" << endl;
			return false;
		}
		PointT p;
		p.x = stod(strvec[0]);
		p.y = stod(strvec[1]);
		p.z = stod(strvec[2]);
		myPoint.push_back(p);
	}
	fin.close();

	//转换成pcd
	pointCloud->width = (int)myPoint.size();
	pointCloud->height = 1;
	pointCloud->is_dense = false;
	pointCloud->points.resize(pointCloud->width * pointCloud->height);
	for (int i = 0; i < myPoint.size(); i++)
	{
		pointCloud->points[i].x = myPoint[i].x;
		pointCloud->points[i].y = myPoint[i].y;
		pointCloud->points[i].z = myPoint[i].z;
	}
	cout << "reading file finished! " << endl;
	cout << "There are " << pointCloud->points.size() << " points!" << endl;
	return true;
}


int main()
{
	//1.读取点云
	PointCloudT::Ptr cloud(new PointCloudT);
	readTxtFile("data1.txt", ' ', cloud);

	//2.显示点云
	pcl::visualization::PCLVisualizer viewer("cloud viewer");
	viewer.addPointCloud<PointT>(cloud, "sample");


	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	//3.点云输出
	pcl::PCDWriter writer;
	writer.writeASCII<PointT>("demo.pcd", *cloud);


	system("pause");
	return 0;
}





//int main(int argc, char** argv)
//{
//	//数组转vector
//	//std::vector<float> testdata(levelModelVerts, levelModelVerts+ sizeof(levelModelVerts)/sizeof(float));
//
//	//std::cout << "xxxxx " << testdata.size() << std::endl;
//
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//
//
//	//std::vector<pcl::PointXYZ> finalDatas;
//	//pcl::PointXYZ temp;
//	//for (int i = 0; i < testdata.size(); i += 3)
//	//{
//	//	temp.x = testdata[i];
//	//	temp.y = testdata[i+1];
//	//	temp.z = testdata[i+2];
//	//	finalDatas.push_back(temp);
//	//}
//	//std::cout << "wwwww " << finalDatas.size() << std::endl;
//
//	
//
//
//	// Load input file into a PointCloud<T> with an appropriate type
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCLPointCloud2 cloud_blob;
//	pcl::io::loadPCDFile("data/bunny.pcd", cloud_blob);
//	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
//	
//	//* the data should be available in cloud
//	
//	//pcl::visualization::CloudViewer viewer1("pcd viewer");
//	//viewer1.showCloud(cloud);
//
//	// Normal estimation*
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//设置法线估计对象
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
//	tree->setInputCloud(cloud);//用cloud构造tree对象
//	n.setInputCloud(cloud);//为法线估计对象设置输入点云
//	n.setSearchMethod(tree);//设置搜索方法
//	n.setKSearch(20);//设置k邻域搜素的搜索范围
//	n.compute(*normals);//估计法线
//	//* normals should not contain the point normals + surface curvatures
//
//	// Concatenate the XYZ and normal fields*
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);//
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
//	//* cloud_with_normals = cloud + normals
//
//	// Create search tree*
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
//	tree2->setInputCloud(cloud_with_normals);//利用有向点云构造tree
//
//	// Initialize objects
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
//	pcl::PolygonMesh triangles;//存储最终三角化的网络模型
//
//	// Set the maximum distance between connected points (maximum edge length)
//	gp3.setSearchRadius(0.025);         //设置搜索半径radius，来确定三角化时k一邻近的球半径。
//
//	// Set typical values for the parameters
//	gp3.setMu(2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
//	gp3.setMaximumNearestNeighbors(100);//设置样本点最多可以搜索的邻域数目100 。
//	gp3.setMaximumSurfaceAngle(M_PI / 4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
//	gp3.setMinimumAngle(M_PI / 18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
//	gp3.setMaximumAngle(2 * M_PI / 3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
//	gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。
//
//	// Get result
//	gp3.setInputCloud(cloud_with_normals);//设置输入点云为有向点云
//	gp3.setSearchMethod(tree2);           //设置搜索方式tree2
//	gp3.reconstruct(triangles);           //重建提取三角化
//   // std::cout << triangles;
//	// Additional vertex information
//	std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
//	/*
//	获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
//	其中 NONE 表示未定义，
//	FREE 表示该点没有在 三 角化后的拓扑内，为自由点，
//	COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
//	BOUNDARY 表示该点在三角化后的拓扑边缘，
//	FRINGE 表示该点在 三 角化后的拓扑内，其连接会产生重叠边。
//	*/
//	std::vector<int> states = gp3.getPointStates();
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	//viewer->setBackgroundColor(0, 0, 0);
//	viewer->addPolygonMesh(triangles, "my");
//
//	viewer->addCoordinateSystem(1.0);
//	viewer->initCameraParameters();
//	// 主循环
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//
//	 //	Finish
//	return (0);
//}




// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
