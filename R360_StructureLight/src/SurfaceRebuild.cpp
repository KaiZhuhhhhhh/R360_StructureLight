//
// Created by 朱凯 on 17/4/16.
//http://www.cnblogs.com/li-yao7758258/p/6497446.html

#include "stdafx.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件

void  mls_Filte()//最小二乘法滤波
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("9.pcd", *cloud);

	// 创建 KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;

	// 定义最小二乘实现的对象mls
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);//是否用多项式拟合逼近法线
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);//设置多项式拟合k临近搜索半径

	// Reconstruct
	mls.process(mls_points);//输出结果

	// Save output
	pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
}

