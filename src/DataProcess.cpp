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
	
	// OpenCV's distortion coefficient vector:  the first two radial parameters come first; these are followed by the
	// two tangential parameters --LearnOpenCV3 Chapter19 Page724
	std::vector < std::vector<double>> distorCoeffList = {
						{-0.100789309140485,  0.111481779308653, 0, 0}, // assigning vector like this requires C++11 or higher
						{-0.0800015988407199, 0.00913215876167434, 0, 0},
						{-0.0429979674496634, 0.0565507535722041, 0, 0}, // assigning vector like this requires C++11 or higher
						{-0.0772943133875519, 0.151213590544407, 0, 0}
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
						0.999656680418481,-0.00430227923311327,-0.0258459220787953,
						0.003299334490053675, 0.999244543003685, -0.0387228313696110,
						0.0259929930293835, 0.0386242627212082, 0.998915677443606
	};
	// rotation matrix for camera set in right
	std::vector < double > rotationMatRightVec = {
						0.996293152093159,-0.0317006774801212,-0.0799688823204344,
						0.0294008181914690, 0.999124135322116, -0.0297750584635399,
						0.0808427299263845, 0.0273135362803202, 0.996352559967562
	};
	rotationMatLeft = cv::Mat(3, 3, CV_64F, rotationMatLeftVec.data());
	rotationMatRight = cv::Mat(3, 3, CV_64F, rotationMatRightVec.data());
	std::cout << "Left Camera Set rotation Matrix: \n" << rotationMatLeft << std::endl;
	std::cout << "Right Camera Set rotation Matrix: \n" << rotationMatRight << std::endl;
	// translation matrix for camera set in left
	std::vector<double> translationMatLeftVec =
	{ -2.44566714617093, - 239.720907753974, - 5.59188270254899 };
	// translation matrix for camera set in right
	std::vector<double> translationMatRightVec =
	{ 40.3426341450911, - 264.737756354131,	7.20784279572053 };

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
	cv::Mat R_upper, P_upper, R_lower, P_lower;

	
	cv::stereoRectify(cameraMatrix[0], distorCoeff[0], cameraMatrix[1], distorCoeff[1], cv::Size(2048, 2048), 
		rotationMatLeft, translationMatLeft, R_upper, R_lower, P_upper, P_lower, Q_left, 
		cv::CALIB_ZERO_DISPARITY, -1, cv::Size(2048, 2048), &validROIL, &validROIR);
	std::cout << "disparity map matrix for LEFT cameras: \n" << Q_left << std::endl;
	// 根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx, mapy
	// mapx, mapy这两个映射表接下来可以给remap()函数调用，来校正图像，
	// 使得两幅图像共面并且行对准
	// use CV_32F for function initUndistortRectifyMap
	cv::initUndistortRectifyMap(cameraMatrix[0], distorCoeff[0], R_upper, P_upper, cv::Size(2048, 2048), CV_32F, mapX[0], mapY[0]);
	cv::initUndistortRectifyMap(cameraMatrix[1], distorCoeff[1], R_lower, P_lower, cv::Size(2048, 2048), CV_32F, mapX[1], mapY[1]);
	
	// 计算右边一对相机的map matrix
	//cv::Mat R_upper_, R_lower_, P_upper_, P_lower_, Q_;
	cv::stereoRectify(cameraMatrix[2], distorCoeff[2], cameraMatrix[3], distorCoeff[3], cv::Size(2048, 2048), 
		rotationMatRight, translationMatRight, R_upper, R_lower, P_upper, P_lower, Q_right, 
		cv::CALIB_ZERO_DISPARITY, -1, cv::Size(2048, 2048), &validROIL, &validROIR);
	std::cout << "disparity map matrix for RIGHT cameras: \n" << Q_right << std::endl;
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

	
	cv::Point temp;
	// 双目校正 stereo rectification
	for (int camera = 0; camera < numCameras; camera++)
	{
		for (int marker = 0; marker < 6; marker++)
		{
			temp = cv::Point(points[camera][marker]);
			std::cout << "After adding offset: Camera " << camera << " Marker " << marker << " " << points[camera][marker] << "\t";
			// at<float>(y,x)是因为y指的是n_row, x指的是n_col
			points[camera][marker].x = mapX[camera].at<float>(int(temp.y), int(temp.x));
			points[camera][marker].y = mapY[camera].at<float>(int(temp.y), int(temp.x));
			std::cout << "After mapping" << points[camera][marker] << std::endl;
		}
	}
	// This is slightly quicker, but I'm not sure about the math
	//for (int marker_set = 0; marker_set < numCameras / 2; marker_set++)
	//{
	//	for (int marker = 0; marker < 6; marker++)
	//	{
	//		MarkerPos3D[marker_set][marker].x = (double(points[2 * marker_set][marker].x) - cx) * T / (double(points[2 * marker_set][marker].y) - double(points[2 * marker_set + 1][marker].y));
	//		MarkerPos3D[marker_set][marker].y = (double(points[2 * marker_set][marker].y) - cy) * T / (double(points[2 * marker_set][marker].y) - double(points[2 * marker_set + 1][marker].y));
	//		MarkerPos3D[marker_set][marker].z = fy * T / (double(points[2 * marker_set][marker].y) - double(points[2 * marker_set + 1][marker].y));
	//		//std::cout << "Camera Set " << marker_set << " Marker " << marker << MarkerPos3D[marker_set][marker] << std::endl;
	//	}
	//}
	
	std::vector<cv::Vec3d> disparityVecLeft, disparityVecRight;

	for (int marker = 0; marker < 6; marker++)
	{

		disparityVecLeft.push_back(cv::Vec3d(double(points[0][marker].x),
			double(points[0][marker].y), double(points[0][marker].y) - double(points[1][marker].y)));
		disparityVecRight.push_back(cv::Vec3d(double(points[2][marker].x),
			double(points[2][marker].y), double(points[2][marker].y) - double(points[3][marker].y)));
	}
	std::vector<cv::Vec3d> MarkerPosVecL, MarkerPosVecR;
	cv::perspectiveTransform(disparityVecLeft, MarkerPosVecL, Q_left);
	cv::perspectiveTransform(disparityVecRight, MarkerPosVecR, Q_right);
	
	for (int marker = 0; marker < 6; marker++)
	{
		MarkerPos3D[0][marker] = cv::Point3d(MarkerPosVecL[marker]);
		MarkerPos3D[1][marker] = cv::Point3d(MarkerPosVecR[marker]);
		std::cout << "Camera 0, Marker " << marker << " : " << MarkerPos3D[0][marker] <<"\t";
		std::cout << "Camera 1, Marker " << marker << " : " << MarkerPos3D[1][marker] << std::endl;
	}
	
}

cv::Point3d DataProcess::mapTo3D(int camera_set, cv::Point upper_point, cv::Point lower_point)
{
	cv::Point temp;
	// 双目校正 stereo rectification
	
	temp = cv::Point(upper_point);
	// at<float>(y,x)是因为y指的是n_row, x指的是n_col
	upper_point.x = mapX[camera_set].at<float>(int(temp.y), int(temp.x));
	upper_point.y = mapY[camera_set].at<float>(int(temp.y), int(temp.x));
	temp = cv::Point(lower_point);
	lower_point.x = mapX[camera_set].at<float>(int(temp.y), int(temp.x));
	lower_point.y = mapY[camera_set].at<float>(int(temp.y), int(temp.x));

	cv::Vec3d disparityVec = cv::Vec3d(upper_point.x, upper_point.y, double(upper_point.y)-double(lower_point.y));
	cv::Vec3d MarkerPosVec;
	if (camera_set == 0)
	{
		cv::perspectiveTransform(disparityVec, MarkerPosVec, Q_left);

	}
	else
	{
		cv::perspectiveTransform(disparityVec, MarkerPosVec, Q_right);
	}
	return cv::Point3d(MarkerPosVec);
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
		
		
		//std::cout <<"Camera Set: "<<i<< " hip:   " << hip[i] << "   " << "knee:   " << knee[i] << "   " << "ankle:   " << ankle[i] << std::endl;
	}
	/*knee_file << knee[0] << "," <<knee[1]<< "\n";
	hip_file << hip[0]<<"," << hip[1] << "\n";
	ankle_file << ankle[0] << "," << ankle[1]<< "\n";*/
}

bool DataProcess::exportGaitData()
{
	bool success = true;
	mapTo3D(); // 
	getJointAngle(); // 约4ms

	std::vector<double> joint_angles = { hip[0], hip[1], knee[0], knee[1], ankle[0], ankle[1] };
	if (ema.EMA_established)
	{
		eura_angles = ema.filter(joint_angles);
		//通过句柄向PLC写入数组
		nErr = AdsSyncWriteReq(pAddr, ADSIGRP_SYM_VALBYHND, lHdlVar2, sizeof(eura_angles), &eura_angles[0]);
		//if (nErr) std::cerr << "Error: AdsSyncReadReq: " << nErr << '\n';
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

bool DataProcess::FindWorldFrame(int camera_set, cv::Mat upper,cv::Mat lower)
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
		p0 = mapTo3D(camera_set, corners_upper[0], corners_lower[0]);
		p1 = mapTo3D(camera_set, corners_upper[2], corners_lower[2]);
		p2 = mapTo3D(camera_set, corners_upper[7], corners_lower[7]);
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
		Rotation[camera_set] = rotation;
		Transform[camera_set] = transform;
		GotWorldFrame = true;
		return true;
	}
	else
	{
		GotWorldFrame = false;
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
	assert(M.cols == 3); // "Matrix must have the same col number as point's row number");
	cv::Mat_<float> src(3/*rows*/, 1 /* cols */);

	src(0, 0) = p.x;
	src(1, 0) = p.y;
	src(2, 0) = p.z;

	cv::Mat_<float> dst = M * src; //USE MATRIX ALGEBRA 
	return cv::Point3d(dst(0, 0), dst(1, 0), dst(2,0));
}
