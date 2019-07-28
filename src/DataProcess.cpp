#include "DataProcess.h"
#include <algorithm>
#include <iostream>


DataProcess::DataProcess()
{
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
	for (int i = 0; i <numCameras; i++)
	{
		// j 是marker 的序号
		for (int j = 0; j < 6; j++)
		{
			MarkerPos3D[i][j].x = (2 * double(points[2*i][j].x) - cx) * T /(2 * (double(points[2 * i][j].x) - double(points[2 * i+1][j].x)));
			MarkerPos3D[i][j].y = -(2 * double(points[2 * i][j].y) - cy) * T / (2 * (double(points[2 * i][j].x) - double(points[2 * i + 1][j].x)));
			MarkerPos3D[i][j].z = fx * T / (2 * (double(points[2 * i][j].x) - double(points[2 * i + 1][i].x)));
			std::cout << "Camera Set " << i << " Marker " << j << MarkerPos3D[i][j] << std::endl;
		}
	}
	
}


void DataProcess::getJointAngle()
{
	mapTo3D();
	for (int i = 0; i < 2; i++)
	{
		thigh.x = MarkerPos3D[i][4].x - MarkerPos3D[i][5].x; thigh.y = MarkerPos3D[i][4].y - MarkerPos3D[i][5].y;
		shank.x = MarkerPos3D[i][2].x - MarkerPos3D[i][3].x; shank.y = MarkerPos3D[i][2].y - MarkerPos3D[i][3].y;
		foot.x = MarkerPos3D[i][0].x - MarkerPos3D[i][1].x; foot.y = MarkerPos3D[i][0].y - MarkerPos3D[i][1].y;

		hip = -((atan(thigh.x / abs(thigh.y))) / pi) * 180;
		knee = -((acos((thigh.x * shank.x + thigh.y * shank.y) / (sqrt(thigh.x * thigh.x + thigh.y * thigh.y) * sqrt(shank.x * shank.x + shank.y * shank.y)))) / pi) * 180;
		ankle = ((acos((foot.x * shank.x + foot.y * shank.y) / (sqrt(foot.x * foot.x + foot.y * foot.y) * sqrt(shank.x * shank.x + shank.y * shank.y)))) / pi) * 180 - 90;
		std::cout << "hip:   " << hip << "   " << "knee:   " << knee << "   " << "ankle:   " << ankle << std::endl;
	}
	
}

bool DataProcess::exportGaitData()
{
	bool success = true;
	getJointAngle();
	// Insert your code here
	return success;
}