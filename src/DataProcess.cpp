#include "DataProcess.h"
#include <algorithm>
#include <iostream>


DataProcess::DataProcess() :numCameras(4),GotWorldFrame(false)
{
	cv::Point points[4][6];
	cv::Point3d MarkerPos3D[2][6];
	cv::Mat image;
	double time = 0;
	double hip[2]; // 0 for left, 1 for right
	double knee[2];
	double ankle[2];
}


DataProcess::~DataProcess()
{
}

// void DataProcess::getTime()
// {
// 	SYSTEMTIME sys;
// 	GetLocalTime(&sys);

// 	if (!gettime)
// 	{
// 		gettime = true;
// 	}
// 	else if ((sys.wSecond - second) > 0 || (sys.wSecond - second) == 0)
// 	{
// 		deltat = sys.wSecond - second + (sys.wMilliseconds - millisecond) / 1000;
// 	}
// 	else
// 	{
// 		deltat = sys.wSecond + 60 - second + (sys.wMilliseconds - millisecond) / 1000;
// 	}
// 	time += deltat;
// 	std::cout << "time:  " << time << "  "<<std::endl;
// 	second = sys.wSecond;
// 	millisecond = sys.wMilliseconds;
// }


void DataProcess::mapTo3D()
{
	const double cx = 1124.8;
	const double cy = 1126.0;
	const double fx = 1018.7;
	const double fy = 1002.1;
	const int T = 200;
	// i 是相机组的序号（每一对相机）
	for (int i = 0; i < numCameras/2; i++)
	{
		// j 是marker 的序号
		for (int j = 0; j < 6; j++)
		{
			MarkerPos3D[i][j].x = (2 * double(points[2 * i][j].x) - cx) * T / (2 * (double(points[2 * i][j].x) - double(points[2 * i + 1][j].x)));
			MarkerPos3D[i][j].y = -(2 * double(points[2 * i][j].y) - cy) * T / (2 * (double(points[2 * i][j].x) - double(points[2 * i + 1][j].x)));
			MarkerPos3D[i][j].z = fx * T / (2 * (double(points[2 * i][j].x) - double(points[2 * i + 1][i].x)));
			std::cout << "Camera Set " << i << " Marker " << j << MarkerPos3D[i][j] << std::endl;
		}
	}
}


void DataProcess::getJointAngle()
{
	mapTo3D();
	/* 所有的坐标现在已经转换到自定义坐标系，矢状面是x-z平面， 额状面是y-z平面*/
	for (int i = 0; i < 2; i++)
	{
		thigh[i] = MarkerPos3D[i][1] - MarkerPos3D[i][0];
		shank[i] = MarkerPos3D[i][3] - MarkerPos3D[i][2];
		foot[i] = MarkerPos3D[i][5] - MarkerPos3D[i][4];

		hip[i] = ((atan2(thigh[i].x, abs(thigh[i].z))) / pi) * 180;
		knee[i] = ((acos((thigh[i].x * shank[i].x + thigh[i].z * shank[i].z) / (sqrt(thigh[i].x * thigh[i].x + thigh[i].z * thigh[i].z) * sqrt(shank[i].x * shank[i].x + shank[i].z * shank[i].z)))) / pi) * 180;
		ankle[i] = ((acos((foot[i].x * shank[i].x + foot[i].z * shank[i].z) / (sqrt(foot[i].x * foot[i].x + foot[i].z * foot[i].z) * sqrt(shank[i].x * shank[i].x + shank[i].z * shank[i].z)))) / pi) * 180;
		std::cout << "hip:   " << hip << "   " << "knee:   " << knee << "   " << "ankle:   " << ankle << std::endl;
	}
}

bool DataProcess::exportGaitData()
{
	bool success = true;
	getJointAngle();
	// 此处输出数据给控制系统
	return success;
}

bool DataProcess::FrameTransform()
{

	return false;
} 

bool DataProcess::FindWorldFrame(const cv::Mat upper, const cv::Mat lower)
{
	cv::Mat upper_bgr,lower_bgr;
	cv::cvtColor(upper, upper_bgr, CV_RGB2BGR);
	cv::cvtColor(lower, lower_bgr, CV_RGB2BGR);
	cv::Point3d vector_x, vector_y, vector_z;
	cv::aruco::Dictionary dcitionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
	return true;
}


