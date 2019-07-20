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
	const double cx = 652.1485214233398;
	const double cy = 394.0842399597168;
	const double f = 683.3077785416543;
	const int T = 120;

	for (int i = 0; i < 6; i++)
	{
		MarkerPos3D[i].x=(2*points[0][i].x-cx)*T/(2*(points[0][i].x-points[1][i].x));
		MarkerPos3D[i].y= -(2 * points[0][i].y - cy)*T / (2 * (points[0][i].x - points[1][i].x));
		MarkerPos3D[i].z = f*T / (2 * (points[0][i].x - points[1][i].x));
	}
}


void DataProcess::getJointAngle()
{
	mapTo3D();
	thigh.x = MarkerPos3D[4].x - MarkerPos3D[5].x; thigh.y = MarkerPos3D[4].y - MarkerPos3D[5].y;
	shank.x = MarkerPos3D[2].x - MarkerPos3D[3].x; shank.y = MarkerPos3D[2].y - MarkerPos3D[3].y;
	foot.x = MarkerPos3D[0].x - MarkerPos3D[1].x; foot.y = MarkerPos3D[0].y - MarkerPos3D[1].y;

	hip = -((atan(thigh.x / abs(thigh.y))) / pi) * 180;
	knee = -((acos((thigh.x*shank.x + thigh.y*shank.y) / (sqrt(thigh.x*thigh.x + thigh.y*thigh.y)*sqrt(shank.x*shank.x + shank.y*shank.y)))) / pi) * 180;
	ankle = ((acos((foot.x*shank.x + foot.y*shank.y) / (sqrt(foot.x*foot.x + foot.y*foot.y)*sqrt(shank.x*shank.x + shank.y*shank.y)))) / pi) * 180 - 90;

	std::cout << "MarkerPos3D[0].z: " << MarkerPos3D[0].z << std::endl;
	std::cout<< "hip:   " << hip << "   " << "knee:   " << knee << "   " << "ankle:   " << ankle << std::endl;
	// getTime();
}