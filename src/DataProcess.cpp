#include "DataProcess.h"
#include <algorithm>
#include <iostream>


DataProcess::DataProcess() :numCameras(4),GotWorldFrame(false)
{
	numCameras = 4;
	GotWorldFrame = false;
	time = 0;
	knee_file.open("knee.csv");
	hip_file.open("hip.csv");
	ankle_file.open("ankle.csv");
	//offset = { cv::Point(500, 500), cv::Point(500,200), cv::Point(750,500), cv::Point(800,200) };
}


DataProcess::~DataProcess()
{
	knee_file.close();
	hip_file.close();
	ankle_file.close();
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
	
	// i 是相机组的序号（每一对相机）
	//TODO: 自定义的矩阵乘法较慢，多次遍历也比较花时间，最好写成opencv自带的矩阵乘法
	for (int i = 0; i < numCameras/2; i++)
	{

		// j 是marker 的序号
		for (int j = 0; j < 6; j++)
		{
			/*std::cout << "points before: " << points[2*i][j] << std::endl;
			points[2 * i][j] = cv::Point(offset[2 * i].x+ points[2 * i][j].x, offset[2 * i].y + points[2 * i][j].y);
			points[2 * i + 1][j] = cv::Point(offset[2 * i+1] + points[2 * i+1][j]);
			std::cout << "points after: " << points[2 * i][j] << std::endl;*/
			MarkerPos3D[i][j].x = ( double(points[2 * i][j].x) - cx) * T / ((double(points[2 * i][j].y) - double(points[2 * i + 1][j].y))/* + delta_cy*/);
			MarkerPos3D[i][j].y = (double(points[2 * i][j].y) - cy) * T / ((double(points[2 * i][j].y) - double(points[2 * i + 1][j].y))/* + delta_cy*/);
			MarkerPos3D[i][j].z = fy * T / ((double(points[2 * i][j].y) - double(points[2 * i + 1][i].y))/* + delta_cy*/);
			/*MarkerPos3D[i][j] = MarkerPos3D[i][j] + Transform[i];
			MarkerPos3D[i][j] = Rotation[i] * MarkerPos3D[i][j];*/
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
		std::cout <<"Camera Set: "<<i<< " hip:   " << hip[i] << "   " << "knee:   " << knee[i] << "   " << "ankle:   " << ankle[i] << std::endl;
	}
	knee_file << knee[0] << "," <<knee[1]<< "\n";
	hip_file << hip[0]<<"," << hip[1] << "\n";
	ankle_file << ankle[0] << "," << ankle[1]<< "\n";
}

bool DataProcess::exportGaitData()
{
	bool success = true;
	getJointAngle();

	return success;
}

bool DataProcess::FrameTransform()
{

	return false;
} 

bool DataProcess::FindWorldFrame(cv::Mat upper,cv::Mat lower, int camera_set)
{
	cv::Mat upper_bgr,lower_bgr;
	cv::cvtColor(upper, upper_bgr, CV_RGB2BGR);
	cv::cvtColor(lower, lower_bgr, CV_RGB2BGR);
	cv::Size boardsize(3, 3);
	cv::Point3d vector_x, vector_y, vector_z, p0, p1,p2;
	std::vector<cv::Point3d> vectors(3);
	std::vector<cv::Point2d> corners_upper, corners_lower;
	/*cv::namedWindow("upper", 0);
	cv::imshow("upper", upper_bgr);
	cv::waitKey(0);*/
	
	/*std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
	cv::aruco::detectMarkers(upper_bgr, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
	cv::aruco::drawDetectedMarkers(upper_bgr, markerCorners, markerIds);*/
	
	bool found = cv::findChessboardCorners(upper_bgr, boardsize, corners_upper);
	std::cout << "found" << found << std::endl;
	//cv::drawChessboardCorners(upper_bgr, boardsize, corners_upper, found);
	found  = cv::findChessboardCorners(lower_bgr, boardsize, corners_lower) && found;
	if (found)
	{
		std::cout << corners_upper << std::endl;
		p0.x = (corners_upper[0].x - cx) * T / (corners_upper[0].y - corners_lower[0].y);
		p0.y = (corners_upper[0].y - cy) * T / (corners_upper[0].y - corners_lower[0].y);
		p0.z = fy * T / (corners_upper[0].y - corners_lower[0].y);
		p1.x = (corners_upper[2].x - cx) * T / (corners_upper[2].y - corners_lower[2].y);
		p1.y = (corners_upper[2].y - cy) * T / (corners_upper[2].y - corners_lower[2].y);
		p1.z = fy * T / (corners_upper[2].y - corners_lower[2].y);
		p2.x = (corners_upper[7].x - cx) * T / (corners_upper[7].y - corners_lower[7].y);
		p2.y = (corners_upper[7].y - cy) * T / (corners_upper[7].y - corners_lower[7].y);
		p2.z = fy * T / (corners_upper[7].y - corners_lower[7].y);
		vectors[1] = p1 - p0;
		cv::Point3d transform = -p0;
		vectors[2] = p2 - p0;
		vectors[0] = crossing(vectors[2], vectors[1]);
		vectors[2] = crossing(vectors[0], vectors[1]);
		for (int i = 0; i < 3; i++)
		{
			vectors[i] = scale(vectors[i]);
		}
		// 相机坐标系到自定义坐标系的转换矩阵
		cv::Mat rotation = cv::Mat(3, 3, CV_64FC1, vectors.data()); // 应该是转置后求逆的，但是他是正交矩阵所以不需
		//std::cout << vectors[0] << std::endl;
		std::cout << "rotation matrix:\n"<<rotation << std::endl;
		std::cout << "transform matrix:\n" << transform << std::endl;
		GotWorldFrame = true;
		Rotation.push_back(rotation);
		Transform.push_back(transform);
		return true;
	}
	else
	{
		return false;
	}
}

// 计算叉乘
cv::Point3d crossing(cv::Point3d u, cv::Point3d v)
{
	return cv::Point3d(u.y * v.z - v.y * u.z, u.z * v.x - v.z * u.x, u.x * v.y - u.y * v.x);
}
// 归一化
cv::Point3d scale(cv::Point3d u)
{
	double length = std::sqrt(pow(u.x, 2)+ pow(u.y,2)+pow(u.z,2));
	return u/length;
}

cv::Point3d operator*(cv::Mat M, cv::Point3d p)
{
	assert(M.cols == 3, "Matrix must have the same col number as point's row number");
	cv::Mat_<double> src(3/*rows*/, 1 /* cols */);

	src(0, 0) = p.x;
	src(1, 0) = p.y;
	src(2, 0) = p.z;

	cv::Mat_<double> dst = M * src; //USE MATRIX ALGEBRA 
	return cv::Point3d(dst(0, 0), dst(1, 0), dst(2,0));
}
