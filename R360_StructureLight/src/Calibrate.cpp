
#include "stdafx.h"
#include "Calibrate.h"
#include <highgui.h> 
#include "cv.h"
#include <iostream> 
#include "opencv2/opencv.hpp"

#include <Windows.h>
#include <stdlib.h>
using namespace std;

CvSize board_size = cvSize(7, 10);   //标定板角点数
CvSize2D32f square_size = cvSize2D32f(10, 10);                //方格长宽

int Camera_ID = 0;
int Cali_Pic_Num;

CvMat * intrinsic_matrix = cvCreateMat(3, 3, CV_64FC1);                //内参数矩阵
CvMat * distortion_coeffs = cvCreateMat(5, 1, CV_64FC1);        //畸变系数
CvMat * extrinsic_matrix = cvCreateMat(4, 4, CV_32FC1);        //畸变系数

vector<CvMat> T_mat_4x4;					//旋转矩阵

void inputCameraParam(CvMat * intrinsic_matrix1, CvMat * distortion_coeffs1, CvMat * extrinsic_matrix1)
{
	CvMat * rotation_vec = cvCreateMat(3, 1, CV_32FC1);                //旋转矩阵
	CvMat * translation_vec = cvCreateMat(3, 1, CV_32FC1);        //平移矩阵
	CvMat *temp = cvCreateMat(2, 3, CV_64FC1);        

	CvFileStorage *fs;
	fs = cvOpenFileStorage("D:/TexasInstruments-DLP/DLP4500-structurelight-R360/bin/calibration/data/camera.xml", 0, CV_STORAGE_READ);
	if (fs)
	{
		*intrinsic_matrix1 = *cvCloneMat((CvMat *)cvReadByName(fs, NULL, "intrinsic"));
		*distortion_coeffs1 = *cvCloneMat((CvMat *)cvReadByName(fs, NULL, "distortion"));//深拷贝，否则在跳出函数后被释放
		*temp = *cvCloneMat((CvMat *)cvReadByName(fs, NULL, "extrinsic"));
		cvReleaseFileStorage(&fs);
	}
	else
	{
		cout << "Error: can not find the intrinsics!!!!!" << endl;
	}

	for (int i = 0; i < 3; i++)
	{
		CV_MAT_ELEM(*rotation_vec, float, i, 0) = CV_MAT_ELEM(*temp, double, 0, i);
		CV_MAT_ELEM(*translation_vec, float, i, 0) = CV_MAT_ELEM(*temp, double, 1, i);
	}

	caculate_Tmat(rotation_vec, translation_vec, extrinsic_matrix1);

	cv::Mat a;
	a = intrinsic_matrix1;
	std::cout << "intrinsic:" << a << endl;
	a = distortion_coeffs1;
	std::cout << "distortion:" << a << endl;
	a = extrinsic_matrix1;
	std::cout <<"extrinsic:"<< a << endl;

	cvReleaseMat(&rotation_vec);
	cvReleaseMat(&translation_vec);
	cvReleaseMat(&temp); 
}

void caculate_Tmat(CvMat * r_vec, CvMat * t_vec, CvMat * T_mat)
{
	CvMat * r_mat = cvCreateMat(3, 3, CV_32FC1);
	cvRodrigues2(r_vec, r_mat);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			CV_MAT_ELEM(*T_mat, float, i, j) = CV_MAT_ELEM(*r_mat, float, i, j);
			CV_MAT_ELEM(*T_mat, float, 3, j) = 0;
		}
		CV_MAT_ELEM(*T_mat, float, i, 3) = CV_MAT_ELEM(*t_vec, float, i, 0);
	}
	CV_MAT_ELEM(*T_mat, float, 3, 3) = 1;

	cvReleaseMat(&r_mat);
}

void rotate_R360Plant(unsigned int n)
{
	HANDLE hcom;
	hcom = CreateFile(_T("COM4"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING
		, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hcom == INVALID_HANDLE_VALUE)
	{
		std::cout<<"连接失败 "<<endl;
	}
	SetupComm(hcom, 1024, 1024);
	DCB dcb;
	GetCommState(hcom, &dcb);
	dcb.BaudRate = 4800;
	dcb.ByteSize = 8;
	dcb.Parity = 0;
	dcb.StopBits = 1;
	SetCommState(hcom, &dcb);

	char data[1];
	data[0] = (char)n;
//	data[1] = 'a';

	DWORD dwWrittenLen = 0;
	if (!WriteFile(hcom, data, 1, &dwWrittenLen, NULL))
	{
		std::cout << "发送失败 "<<endl;
	}

	_sleep(1000);

	char str[2];
	char aa, bb;
	DWORD wCount;//读取的字节数		 
	BOOL bReadStat;
	bReadStat = ReadFile(hcom, str, 1, &wCount, NULL);
	if (!bReadStat)
	{
		std::cout << "读取失败 "<<endl;
	}
	aa = str[0];
	//	bb=str[1];
	std::cout << "接收:"<<aa;
	//	dlp::CmdLine::Print("接收:",bb);

	CloseHandle(hcom);  //关闭通讯端口

}



int find_rotation_mat()
{
	char  cali_flag;
	char t[10];
	cout << "1、进行标定    2、已标定，直接测量" << endl;
	cin >> cali_flag;

	cout << "输入一周旋转几次：" << endl;
	cin >> t;

	int number_image = 1;
	char *str1;
	str1 = ".jpg";
	char filename[20] = "";

	float square_length = square_size.width;                //方格长度
	float square_height = square_size.height;                //方格高度
	int board_width = board_size.width;   //每行角点数
	int board_height = board_size.height;  //每列角点数
	int total_per_image = board_width*board_height;  //每张图片角点总数

	if (cali_flag == '1')
	{
		CvCapture* capture;
		capture = cvCreateCameraCapture(Camera_ID);

		if (capture == 0)
		{
			printf("无法捕获摄像头设备！\n\n");
			return 0;
		}
		else
		{
			printf("捕获摄像头设备成功！！\n\n");
		}

		IplImage* frame = NULL;

		cvNamedWindow("摄像机帧截取窗口", 1);

		printf("按“C”键截取当前帧并保存为标定图片...\n按“Q”键退出截取帧过程...\n\n");


		while (true)
		{
			frame = cvQueryFrame(capture);
			if (!frame)
				break;
			cvShowImage("摄像机帧截取窗口", frame);

			if (cvWaitKey(10) == 'c')
			{
				sprintf_s(filename, "%d.jpg", number_image);
				cvSaveImage(filename, frame);
				cout << "成功获取当前帧，并以文件名" << filename << "保存...\n\n";
				printf("按“C”键截取当前帧并保存为标定图片...\n按“Q”键退出截取帧过程...\n\n");
				number_image++;
				rotate_R360Plant(atoi(t));
			}
			else if (cvWaitKey(10) == 'q')
			{
				printf("截取图像帧过程完成...\n\n");
				cout << "共成功截取" << --number_image << "帧图像！！\n\n";
				break;
			}
		}

		cvDestroyWindow("摄像机帧截取窗口");
		//	cvReleaseImage(&frame);
		cvReleaseCapture(&capture);
	}
	else if (cali_flag == '2')
	{
		number_image = Cali_Pic_Num;
	}

	IplImage * show;
	cvNamedWindow("RePlay", 1);

	int number_image_copy = number_image;  //复制图像帧数

	int count;  //存储每帧图像中实际识别的角点数
	int found;        //识别标定板角点的标志位
	int step;        //存储步长，step=successes*total_per_image;
	int successes = 0;        //存储成功找到标定板上所有角点的图像帧数
	int a = 1;        //临时变量，表示在操作第a帧图像


	CvPoint2D32f * image_points_buf = new CvPoint2D32f[total_per_image];   //存储角点图像坐标的数组
	CvMat * image_points = cvCreateMat(number_image*total_per_image, 2, CV_32FC1);        //存储角点的图像坐标的矩阵                
	CvMat * object_points = cvCreateMat(number_image*total_per_image, 3, CV_32FC1);        //存储角点的三维坐标的矩阵
	CvMat * point_counts = cvCreateMat(number_image, 1, CV_32SC1);                //存储每帧图像的识别的角点数

	while (a <= number_image_copy)
	{
		sprintf_s(filename, "%d.jpg", a);
		show = cvLoadImage(filename, -1);

		found = cvFindChessboardCorners(show, board_size, image_points_buf, &count,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found == 0)
		{
			cout << "第" << a << "帧图片无法找到棋盘格所有角点!\n\n";
			cvNamedWindow("RePlay", 1);
			cvShowImage("RePlay", show);
			cvWaitKey(0);

		}
		else
		{
			cout << "第" << a << "帧图像成功获得" << count << "个角点...\n";

			cvNamedWindow("RePlay", 1);

			IplImage * gray_image = cvCreateImage(cvGetSize(show), 8, 1);
			cvCvtColor(show, gray_image, CV_BGR2GRAY);
			cout << "获取源图像灰度图过程完成...\n";
			cvFindCornerSubPix(gray_image, image_points_buf, count, cvSize(11, 11), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cout << "灰度图亚像素化过程完成...\n";
			cvDrawChessboardCorners(show, board_size, image_points_buf, count, found);
			cout << "在源图像上绘制角点过程完成...\n\n";
			cvShowImage("RePlay", show);

			cvWaitKey(0);
		}

		if (total_per_image == count)//找到了所有角点
		{
			step = successes*total_per_image;
			for (int i = step, j = 0; j < total_per_image; ++i, ++j)
			{
				CV_MAT_ELEM(*image_points, float, i, 0) = image_points_buf[j].x;
				CV_MAT_ELEM(*image_points, float, i, 1) = image_points_buf[j].y;
				CV_MAT_ELEM(*object_points, float, i, 0) = (float)((j / board_width)*square_length);//行
				CV_MAT_ELEM(*object_points, float, i, 1) = (float)((j%board_width)*square_height);//列
				CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
			}
			CV_MAT_ELEM(*point_counts, int, successes, 0) = total_per_image;
			successes++;
		}
		a++;
	}
	CvSize Pic_size = cvGetSize(show);

	cvReleaseImage(&show);
	cvDestroyWindow("RePlay");


	cout << "*********************************************\n";
	cout << number_image << "帧图片中，标定成功的图片为" << successes << "帧...\n";
	cout << number_image << "帧图片中，标定失败的图片为" << number_image - successes << "帧...\n\n";
	cout << "*********************************************\n\n";

	CvMat * object_points2 = cvCreateMat(total_per_image, 3, CV_32FC1);
	CvMat * image_points2 = cvCreateMat(total_per_image, 2, CV_32FC1);
	CvMat * point_counts2 = cvCreateMat(successes, 1, CV_32SC1);

	CvMat * rotation_vec = cvCreateMat(3, 1, CV_32FC1);                //旋转矩阵
	CvMat * translation_vec = cvCreateMat(3, 1, CV_32FC1);        //平移矩阵
	CvMat * T_Mat = cvCreateMat(4, 4, CV_32FC1);        //平移矩阵

	for (int j = 0; j < successes; ++j)
	{
		for (int i = 0; i < total_per_image; ++i)
		{
			CV_MAT_ELEM(*image_points2, float, i, 0) = CV_MAT_ELEM(*image_points, float, i + j*total_per_image, 0);
			CV_MAT_ELEM(*image_points2, float, i, 1) = CV_MAT_ELEM(*image_points, float, i + j*total_per_image, 1);
			CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 0);
			CV_MAT_ELEM(*object_points2, float, i, 1) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 1);
			CV_MAT_ELEM(*object_points2, float, i, 2) = CV_MAT_ELEM(*object_points, float, i + j*total_per_image, 2);
		}
		CV_MAT_ELEM(*point_counts2, int, j, 0) = CV_MAT_ELEM(*point_counts, int, j, 0);

		cvFindExtrinsicCameraParams2(object_points2, image_points2, intrinsic_matrix, distortion_coeffs, rotation_vec, translation_vec);
		caculate_Tmat(rotation_vec, translation_vec, T_Mat);//生成齐次变换
		T_mat_4x4.push_back(*cvCloneMat(T_Mat));
	}

	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);

	cvReleaseMat(&rotation_vec);
	cvReleaseMat(&translation_vec);
	cvReleaseMat(&T_Mat);
}
