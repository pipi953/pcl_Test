// pcl_Test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//点的类型的头文件
#include <pcl/point_types.h>	
//点云文件IO（pcd文件）
#include <pcl/io/pcd_io.h>	
//kd树
#include <pcl/kdtree/kdtree_flann.h>
//特征提取
#include <pcl/features/normal_3d.h>
//重构
#include <pcl/surface/gp3.h>
//可视化
#include <pcl/visualization/pcl_visualizer.h>
//多线程
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
//	数据
#include "data.h"


#include <iostream>
#include <fstream>
#include <strstream>
#include <vector>

//	泊松表面重建
#include <pcl/filters/passthrough.h>
//特征提取
#include <pcl/features/normal_3d_omp.h>
//重构
#include <pcl/surface/poisson.h>
//点云文件IO（ply文件）
#include <pcl/io/ply_io.h>



typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//PointCloudT::Ptr  = 
//pcl::PointCloud<pcl::PointXYZ>::Ptr


bool readTxtFile(const std::string &fileName, const char tag, const PointCloudT::Ptr &pointCloud)
{
	cout << "reading file start..... " << endl;
	ifstream fin(fileName);
	std::string linestr;
	std::vector<PointT> myPoint;
	while (std::getline(fin, linestr))
	{
		std::vector<std::string> strvec;
		std::string s;
		std::stringstream ss(linestr);
		while (std::getline(ss, s, tag))
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

bool readModelData_ArrayToPcd(float* dataArr, int dataSize, const PointCloudT::Ptr &pointCloud)
{
	cout << "reading file start..... " << endl;
	std::vector<PointT> myPoint;

	//数组转vector
	std::vector<float> dataVec(dataArr, dataArr + dataSize);
	std::cout << "xxxxx " << dataVec.size() << std::endl;

	PointT p;

	for (int i = 0; i < dataVec.size(); i += 3)
	{
		p.x = dataVec[i];
		p.y = dataVec[i+1];
		p.z = dataVec[i+2];
		myPoint.push_back(p);
	}


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
	PointCloudT::Ptr cloud1(new PointCloudT);
	//readTxtFile("data1.txt", ' ', cloud1);
	readModelData_ArrayToPcd(levelModelVerts, sizeof(levelModelVerts) / sizeof(float), cloud1);

	/*
	//2.显示点云
	pcl::visualization::PCLVisualizer viewer1("cloud viewer");
	viewer1.addPointCloud<PointT>(cloud1, "sample");

	while (!viewer1.wasStopped())
	{
		viewer1.spinOnce();
	}
	*/
	/*
	//3.点云输出，写入本地
	pcl::PCDWriter writer;
	writer.writeASCII<PointT>("demo.pcd", *cloud1);
	*/

	/*
	//	直接加载pcd模型
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("data/bunny.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	
	//	显示点云图像
	pcl::visualization::CloudViewer viewer1("pcd viewer");
	viewer1.showCloud(cloud);
	*/


	/*	
	//	--------------------------	贪婪投影三角化算法 --------------------------	
	// 法向量估算 *
	pcl::NormalEstimation<PointT, pcl::Normal> n;		//	设置法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);	//	存储估计的法线
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);				//	定义kd树指针
	tree->setInputCloud(cloud1);		//	用cloud构造tree对象
	n.setInputCloud(cloud1);				//	为法线估计对象设置输入点云
	n.setSearchMethod(tree);			//	设置搜索方法
	n.setKSearch(20);						//	设置k邻域搜素的搜索范围
	n.compute(*normals);				//	估计法线
	//* 法线不应包含点法线 + 表面曲率

	//	将点云位姿，颜色，法线信息连接到一起。（连接 XYZ 和 法向量字段）
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud1, *normals, *cloud_with_normals);	//	连接字段，cloud_with_normals存储有向点云
	//* cloud_with_normals = cloud + normals

	// 定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);	//	定义搜索树对象
	tree2->setInputCloud(cloud_with_normals);	//	利用有向点云构造tree

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;	//	定义三角化对象
	pcl::PolygonMesh triangles;	//	存储最终三角化的网络模型

	// 设置三角化参数。
	gp3.setSearchRadius(30000.0f);  //	设置搜索半径radius，也就是KNN的球半径（来确定三角化时k一邻近的球半径）
	//	设置样本点搜索其紧邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMu(2.8);   //	设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);	//	设置样本点最多可以搜索的邻域个数，典型值是50 -100 。
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //	45度，设置某点法线方向偏离样本点发现的最大角度为45度，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
	gp3.setMinimumAngle(M_PI / 18);        //	10度，设置三角化后得到的三角形内角的最小的角度为45度       -- 设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
	gp3.setMaximumAngle(2 * M_PI / 3);       //	120度，设置三角化后得到的三角形内角的最大角度为120度   -- 设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
	gp3.setNormalConsistency(false);     //	设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。

	// 获取结果
	gp3.setInputCloud(cloud_with_normals);	//	设置输入点云为有向点云
	gp3.setSearchMethod(tree2);					//	设置搜索方式为 tree2
	gp3.reconstruct(triangles);						//	重建提取三角化
   // std::cout << triangles;
	// 附加顶点信息
	std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
	
	//获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
	//其中 NONE 表示未定义，
	//FREE 表示该点没有在 三 角化后的拓扑内，为自由点，
	//COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
	//BOUNDARY 表示该点在三角化后的拓扑边缘，
	//FRINGE 表示该点在 三 角化后的拓扑内，其连接会产生重叠边。
	
	std::vector<int> states = gp3.getPointStates();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(triangles, "my");

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	// 主循环
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	//3.点云输出
	pcl::PCDWriter writer;
	writer.writeASCII<PointT>("bemo.pcd", *cloud1);
	*/


	//	泊松表面重建算法

	// 计算法向量
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //法向量点云对象指针
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud1);
	n.setInputCloud(cloud1);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //计算法线，结果存储在normals中

	//	将点云和法线放到一起
	pcl::concatenateFields(*cloud1, *normals, *cloud_with_normals);

	//	创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	//创建Poisson对象，并设置参数
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(20); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度
	//pn.setIndices();

	//设置搜索方法和输入点云
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	//创建多变形网格，用于存储结果
	pcl::PolygonMesh mesh;
	//执行重构
	pn.performReconstruction(mesh);

	//保存网格图
	pcl::io::savePLYFile("result.ply", mesh);

	// 显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(mesh, "my");
	viewer->addCoordinateSystem(50.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	system("pause");
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
