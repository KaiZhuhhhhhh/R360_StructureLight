// 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "Calibrate.h"

#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>  

#include <cv.h>
#include <highgui.h>
using namespace std;

int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:  
	user_data++;
}

int _tmain(int argc, _TCHAR* argv[])
{
	IplImage * test;
	test = cvLoadImage("D:\\Sample_8.png");//图片路径
	cvNamedWindow("test_demo", 1);
	cvShowImage("test_demo", test);
	cvWaitKey(0);
	cvDestroyWindow("test_demo");
	cvReleaseImage(&test);
	



	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile("my_point_cloud.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");



	//blocks until the cloud is actually rendered  
	viewer.showCloud(cloud);

	//use the following functions to get access to the underlying more advanced/powerful  
	//PCLVisualizer  

	//This will only get called once  
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//This will get called once per visualization iteration  
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//you can also do cool processing here  
		//FIXME: Note that this is running in a separate thread from viewerPsycho  
		//and you should guard against race conditions yourself...  
		user_data++;
	}
	return 0;
}

