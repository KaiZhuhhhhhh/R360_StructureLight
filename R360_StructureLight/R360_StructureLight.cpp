// 定义控制台应用程序的入口点。
//关掉了Microsoft符号调试器
//
// Created by 朱凯 on 17/4/4.
//
#include "stdafx.h"
#include "Calibrate.h"
#include "PairAlign.h"
#include "config.h"

#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>  

#include <cv.h>
#include <highgui.h>


int _tmain(int argc, char** argv)
{
	char *str1 = "1_point_cloud.ply";
	argv[1] = str1;
	char *str2 = "2_point_cloud.ply";
	argv[2] = str2;
	char *str3 = "3_point_cloud.ply";
	argv[3] = str3;
	char *str4 = "4_point_cloud.ply";
	argv[4] = str4;
	char *str5 = "5_point_cloud.ply";
	argv[5] = str5;
	char *str6 = "6_point_cloud.ply";
	argv[6] = str6;
	char *str7 = "7_point_cloud.ply";
	argv[7] = str7 ;
	char *str8 = "8_point_cloud.ply";
	argv[8] = str8;
	char *str9 = "9_point_cloud.ply";
	argv[9] = str7;

	ArgvConfig();
	argc = total_clude+1;
	//读取数据
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //模型
	loadData(argc, argv, data); //读取pcd文件数据，定义见上面	

	inputCameraParam(intrinsic_matrix, distortion_coeffs);//从文件中读取相机参数
	find_rotation_mat();//算出每幅标定图像的其次变换矩阵存在全局变量T_mat_4x4中

	AccurateRegistration(data);//精细拼接

	return 0;
}

