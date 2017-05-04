//
// Created by 朱凯 on 17/4/4.
//

//#pragma once

#ifndef PAIR_ALIGN_ZK_H
#define PAIR_ALIGN_ZK_H

#include <string>

#include <boost/make_shared.hpp> //共享指针
//点/点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
//pcd文件输入/输出
#include <pcl/io/pcd_io.h>
//滤波
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
//特征
#include <pcl/features/normal_3d.h>
//配准
#include <pcl/registration/icp.h> //ICP方法
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
//可视化
#include <pcl/visualization/pcl_visualizer.h>

#include "cv.h"

//命名空间
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//定义类型的别名
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//定义结构体，用于处理点云
struct PCD
{
	PointCloud::Ptr cloud; //点云指针
	std::string f_name; //文件名
	//构造函数
	PCD() : cloud(new PointCloud) {}; //初始化
};

extern int total_clude;
extern int Registration_flag;//0:转台 1：icp 2：转台+icp
extern int KSearchnum;
extern float MaxCorrespondenceDistance; //对应点之间的最大距离（0.1）, 在配准过程中，忽略大于该阈值的点
extern float LeafSize;
extern float TransformationEpsilon ;//允许最大误差
extern bool downsample_flag;
extern int GetRough_T_flag;
extern Eigen::Matrix4f GetR360Rough_T;

void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);

void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);

void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false);

void CvMatToMatrix4fzk(Eigen::Matrix4f *pcl_T, CvMat *cv_T);

void roughTranslation(PointCloud::Ptr cloud, Eigen::Matrix4f &T, int n );

void AccurateRegistration(std::vector<PCD, Eigen::aligned_allocator<PCD> > &data_temp);

#endif