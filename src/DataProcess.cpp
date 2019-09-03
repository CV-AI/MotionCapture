#include "DataProcess.hpp"
#include <algorithm>
#include <iostream>


DataProcess::DataProcess() :numCameras(4), GotWorldFrame(false)
{
	numCameras = 4;
	GotWorldFrame = false;
	time = 0;
	hip[0] = 0; hip[1] = 0;
	ankle[0] = 0; ankle[1] = 0;
	knee[0] = 0; knee[1] = 0;
	knee_file.open("knee.csv");
	hip_file.open("hip.csv");
	ankle_file.open("ankle.csv");
	
	// OpenCV's distortion coefficient vector is composed of MATLAB's two tangential distortion coefficients 
	// followed by the two radial distortion coefficients.
	std::vector < std::vector<double>> distorCoeffList = {
						{0, 0, -0.0560974273936180, 0.0820468069671561}, // assigning vector like this requires C++11 or higher
						{0, 0, -0.0702735401089445, 0.102611484500887},
						{0, 0, -0.0560974273936180, 0.0820468069671561},
						{0, 0, -0.0702735401089445, 0.102611484500887}
	};
	// initiate parameters for camera rectification
	for (int camera = 0; camera < numCameras; camera++)
	{
		// use CV_64F type for inputs of function of stereoRectify
		cameraMatrix[camera] = cv::Mat::zeros(3, 3, CV_64F);
		cameraMatrix[camera].at<double>(0, 0) = fx_list[camera];
		cameraMatrix[camera].at<double>(1, 1) = fy_list[camera];
		cameraMatrix[camera].at<double>(0, 2) = cx_list[camera];
		cameraMatrix[camera].at<double>(1, 2) = cy_list[camera];
		cameraMatrix[camera].at<double>(2, 2) = 1;
		// 4 rows, 1 column
		distorCoeff[camera] = cv::Mat(distorCoeffList[camera].size(), 1, CV_64F, distorCoeffList[camera].data());
		std::cout << "Camera " << camera << " Matrix: \n" << cameraMatrix[camera] << std::endl;
		std::cout << "Camera " << camera << " distortion vector: \n" << distorCoeff[camera] << std::endl;
	}
	//
	// initialize rotation and translation matrix for left and right camera set
	// these hard coded parameters are calculated by matlab for camera caliabration
	// rotation matrix for camera set in left
	std::vector < double > rotationMatLeftVec = {
						0.999872825151855,-0.00942487454477512,0.0128648848678506,
						0.00993803474975875, 0.999133141003343, -0.0404252645520858,
						-0.0124727297798192, 0.0405479751480735, 0.999099741128598
	};
	// rotation matrix for camera set in right
	std::vector < double > rotationMatRightVec = {
						0.999872825151855,-0.00942487454477512,0.0128648848678506,
						0.00993803474975875, 0.999133141003343, -0.0404252645520858,
						-0.0124727297798192, 0.0405479751480735, 0.999099741128598
	};
	rotationMatLeft = cv::Mat(3, 3, CV_64F, rotationMatLeftVec.data());
	rotationMatRight = cv::Mat(3, 3, CV_64F, rotationMatRightVec.data());
	std::cout << "Left Camera Set rotation Matrix: \n" << rotationMatLeft << std::endl;
	std::cout << "Right Camera Set rotation Matrix: \n" << rotationMatRight << std::endl;
	// translation matrix for camera set in left
	std::vector<double> translationMatLeftVec =
	{ 9.62562726230605, -244.628114017665, -13.0410750920663 };
	// translation matrix for camera set in right
	std::vector<double> translationMatRightVec =
	{ 9.62562726230605, -244.628114017665, -13.0410750920663 };

	translationMatLeft = cv::Mat(translationMatLeftVec.size(), 1, CV_64F, translationMatLeftVec.data());
	translationMatRight = cv::Mat(translationMatRightVec.size(), 1, CV_64F, translationMatRightVec.data());
	std::cout << "Left Camera Set translation Matrix: \n" << translationMatLeft << std::endl;
	std::cout << "Right Camera Set translation Matrix: \n" << translationMatRight << std::endl;
	
	// 立体校正的时候需要两幅图像共面并且行对准，以使得立体匹配更方便
	// 使的两幅图像共面的方法就是把两个相机平面投影到一个公共的成像平面上，这样每幅图像投影到公共平面
	//  就需要一个旋转矩阵R, stereoRectify()这个函数计算的就是从图像平面投影到公共成像平面的的旋转矩阵Rl,Rr.
	//  RlRr就是左右相机平面共面的校正旋转矩阵，左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面了；
	//  其中Pl Pr为两个相机的校正内参矩阵(3x4,最后一列为0),也可以称为相机坐标系到像素坐标系的投影矩阵，
	//  Q 为像素坐标系与相机坐标系之间的重投影矩阵；
	
	cv::Rect validROIL, validROIR;
	cv::Mat R_upper, P_upper, R_lower, P_lower, Q;

	
	cv::stereoRectify(cameraMatrix[0], distorCoeff[0], cameraMatrix[1], distorCoeff[1], cv::Size(2048, 2048), 
		rotationMatLeft, translationMatLeft, R_upper, R_lower, P_upper, P_lower, Q, 
		cv::CALIB_ZERO_DISPARITY, -1, cv::Size(2048, 2048), &validROIL, &validROIR);
	// 根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx, mapy
	// mapx, mapy这两个映射表接下来可以给remap()函数调用，来校正图像，
	// 使得两幅图像共面并且行对准
	// use CV_32F for function initUndistortRectifyMap
	cv::initUndistortRectifyMap(cameraMatrix[0], distorCoeff[0], R_upper, P_upper, cv::Size(2048, 2048), CV_32F, mapX[0], mapY[0]);
	cv::initUndistortRectifyMap(cameraMatrix[1], distorCoeff[1], R_lower, P_lower, cv::Size(2048, 2048), CV_32F, mapX[1], mapY[1]);
	
	// 计算右边一对相机的map matrix
	//cv::Mat R_upper_, R_lower_, P_upper_, P_lower_, Q_;
	cv::stereoRectify(cameraMatrix[2], distorCoeff[2], cameraMatrix[3], distorCoeff[3], cv::Size(2048, 2048), 
		rotationMatRight, translationMatRight, R_upper, R_lower, P_upper, P_lower, Q, 
		cv::CALIB_ZERO_DISPARITY, -1, cv::Size(2048, 2048), &validROIL, &validROIR);
	cv::initUndistortRectifyMap(cameraMatrix[2], distorCoeff[2], R_upper, P_upper, cv::Size(2048, 2048), CV_32F, mapX[2], mapY[2]);
	cv::initUndistortRectifyMap(cameraMatrix[3], distorCoeff[3], R_lower, P_lower, cv::Size(2048, 2048), CV_32F, mapX[3], mapY[3]);

}


DataProcess::~DataProcess()
{
	knee_file.close();
	hip_file.close();
	ankle_file.close();
}


void DataProcess::mapTo3D()
{

	// i 是相机组的序号（每一对相机）
	//TODO: 自定义的矩阵乘法较慢，多次遍历也比较花时间，最好写成opencv自带的矩阵乘法
	cv::Point temp;
	for (int camera = 0; camera < numCameras; camera++)
	{
		for (int marker = 0; marker < 6; marker++)
		{
			temp = cv::Point(points[camera][marker]);
			// at<float>(y,x)是因为y指的是n_row, x指的是n_col
			points[camera][marker].x = mapX[camera].at<float>(int(temp.y), int(temp.x));
			points[camera][marker].y = mapY[camera].at<float>(int(temp.y), int(temp.x));
		}
	}
	for (int marker_set = 0; marker_set < numCameras / 2; marker_set++)
	{
		for (int marker = 0; marker < 6; marker++)
		{

			MarkerPos3D[marker_set][marker].x = (float(points[2 * marker_set][marker].x) - cx) * T / (float(points[2 * marker_set][marker].y) - float(points[2 * marker_set + 1][marker].y));
			MarkerPos3D[marker_set][marker].y = (float(points[2 * marker_set][marker].y) - cy) * T / (float(points[2 * marker_set][marker].y) - float(points[2 * marker_set + 1][marker].y));
			MarkerPos3D[marker_set][marker].z = fy * T / (float(points[2 * marker_set][marker].y) - float(points[2 * marker_set + 1][marker].y));
			std::cout << "Camera Set " << marker_set << " Marker " << marker << MarkerPos3D[marker_set][marker] << std::endl;
		}
	}
}


void DataProcess::getJointAngle()
{
	
	// 所有的坐标现在已经转换到自定义坐标系，矢状面是x-z平面， 额状面是y-z平面
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
	mapTo3D();
	getJointAngle();

	std::vector<float> joint_angles = { hip[0], hip[1], knee[0], knee[1], ankle[0], ankle[1] };
	if (ema.EMA_established)
	{
		eura_angles = ema.filter(joint_angles);
		//通过句柄向PLC写入数组
		nErr = AdsSyncWriteReq(pAddr, ADSIGRP_SYM_VALBYHND, lHdlVar2, sizeof(eura_angles), &eura_angles[0]);
		if (nErr) std::cerr << "Error: AdsSyncReadReq: " << nErr << '\n';
	}
	else
	{
		ema.feed(joint_angles);
	}
	return success;
}

bool DataProcess::FrameTransform()
{

	return false;
} 

bool DataProcess::FindWorldFrame(cv::Mat upper,cv::Mat lower)
{
	cv::Mat upper_bgr,lower_bgr;
	cv::cvtColor(upper, upper_bgr, CV_RGB2BGR);
	cv::cvtColor(lower, lower_bgr, CV_RGB2BGR);
	cv::Size boardsize(3, 3);
	cv::Point3d vector_x, vector_y, vector_z, p0, p1,p2;
	std::vector<cv::Point3d> vectors(3);
	std::vector<cv::Point2d> corners_upper, corners_lower;
	
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
	float length = std::sqrt(pow(u.x, 2)+ pow(u.y,2)+pow(u.z,2));
	return u/length;
}

cv::Point3d operator*(cv::Mat M, cv::Point3d p)
{
	assert(M.cols == 3, "Matrix must have the same col number as point's row number");
	cv::Mat_<float> src(3/*rows*/, 1 /* cols */);

	src(0, 0) = p.x;
	src(1, 0) = p.y;
	src(2, 0) = p.z;

	cv::Mat_<float> dst = M * src; //USE MATRIX ALGEBRA 
	return cv::Point3d(dst(0, 0), dst(1, 0), dst(2,0));
}
