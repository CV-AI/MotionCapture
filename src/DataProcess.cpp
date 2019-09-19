#include "DataProcess.hpp"
#include <algorithm>
#include <iostream>

DataProcess::DataProcess() :numCameras(4), GotWorldFrame(false)
{
	GotWorldFrame = false;
	joint_angles = { 0,0,0,0,0,0 };
	joint_angles_pre = { 0,0,0,0,0,0 };
	
	angles_file.open("angles.csv");
	eura_file.open("eura.csv");
	// OpenCV's distortion coefficient vector:  the radial parameters come first; these are followed by the
	// two tangential parameters --LearnOpenCV3 Chapter19 Page724
	std::vector < std::vector<double>> distorCoeffList = {
						{-0.0709573236367611, 0.0819347816184623, -0.00229710720816990,	0.00223004557805238}, // assigning vector like this requires C++11 or higher
						{-0.0782886283658884, 0.129966473775231, -0.00447759677730269, 0.00371640574661551},
						{-0.0637950488364343, 0.115821950263960, 0.00448871862274016, 0.00244843774244850}, // assigning vector like this requires C++11 or higher
						{-0.0732969283921288, 0.139401736087819, 0.00608572295729794, 0.00261382520826605}
	};
	// initiate parameters for camera rectification
	for (int camera = 0; camera < numCameras; camera++)
	{
		// use CV_64F type for inputs of function of stereoRectify
		cameraMatrix[camera] = cv::Matx33d::zeros();
		cameraMatrix[camera](0, 0) = fx_list[camera];
		cameraMatrix[camera](1, 1) = fy_list[camera];
		cameraMatrix[camera](0, 2) = cx_list[camera];
		cameraMatrix[camera](1, 2) = cy_list[camera];
		cameraMatrix[camera](2, 2) = 1;
		// 4 rows, 1 column
		distorCoeff[camera] = cv::Mat(distorCoeffList[camera].size(), 1, CV_64F, distorCoeffList[camera].data());
		std::cout << "Camera " << camera << " Matrix: \n" << cameraMatrix[camera] << std::endl;
		std::cout << "Camera " << camera << " distortion vector: \n" << distorCoeff[camera] << std::endl;
	}
	//
	// initialize rotation and translation matrix for left and right camera set
	// these hard coded parameters are calculated by matlab for camera caliabration
	// rotation matrix for camera set in left
	// the matlab matrix need to be transposed to fit opencv
	std::vector < double > rotationMatLeftVec = {
						0.999950740825703, -0.00839254784768728, -0.00529915679631496,
						0.00847312121061239,	0.999845970938895,	0.0153701208261035,
						0.00516934609771118, -0.0154142641044920,	0.999867830427122
	};
	// rotation matrix for camera set in right
	std::vector < double > rotationMatRightVec = {
						0.999220726328862,	0.0288498130258020, -0.0269374899201554,
						-0.0280613789593659,	0.999179560790386,	0.0292021285119807,
						0.0277578652947424, -0.0284234689492265,	0.999210492002146
	};
	rotationMatLeft = cv::Mat(3, 3, CV_64F, rotationMatLeftVec.data()).t();
	rotationMatRight = cv::Mat(3, 3, CV_64F, rotationMatRightVec.data()).t();
	std::cout << "Left Camera Set rotation Matrix: \n" << rotationMatLeft << std::endl;
	std::cout << "Right Camera Set rotation Matrix: \n" << rotationMatRight << std::endl;
	// translation matrix for camera set in left
	std::vector<double> translationMatLeftVec =
	{ 1.09924022464804,-311.013107025229, -0.325991832090340 };
	// translation matrix for camera set in right
	std::vector<double> translationMatRightVec =
	{ 10.6279676948522, -264.663517556467, -9.95862862688720 };

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


	// 双目标定
	cv::stereoRectify(cameraMatrix[0], distorCoeff[0], cameraMatrix[1], distorCoeff[1], cv::Size(2048, 2048),
		rotationMatLeft, translationMatLeft, Rectify[0], Rectify[1], Projection[0], Projection[1], Q_left,
		cv::CALIB_ZERO_DISPARITY, -1, cv::Size(2048, 2048), &validROIL, &validROIR);
	std::cout << "disparity map matrix for LEFT cameras: \n" << Q_left << std::endl;

	cv::stereoRectify(cameraMatrix[2], distorCoeff[2], cameraMatrix[3], distorCoeff[3], cv::Size(2048, 2048),
		rotationMatRight, translationMatRight, Rectify[2], Rectify[3], Projection[2], Projection[3], Q_right,
		cv::CALIB_ZERO_DISPARITY, -1, cv::Size(2048, 2048), &validROIL, &validROIR);
	std::cout << "disparity map matrix for RIGHT cameras: \n" << Q_right << std::endl;
	cv::FileStorage fs("FrameDefine.yml", cv::FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R0"] >> Rotation[0];
		fs["R1"] >> Rotation[1];
		fs["T0"] >> Transform[0];
		fs["T1"] >> Transform[1];
		GotWorldFrame = true;
		std::cout << "----!!!!! Use file to initialize Rotation and Transform matrix -----!!!!!" << std::endl;
	}
	// 可以从文件里读取相机标定文件，但是OpenCV的标定很不流畅，目前还是使用Matlab标定
	/*cv::FileStorage fs("calib.yml", cv::FileStorage::READ);
	fs["Q0"] >> Q_left;
	std::cout << "Q_left" << Q_left;
	fs["Q1"] >> Q_right;
	fs.release();*/
}


DataProcess::~DataProcess()
{
	// 打开的文件一定要关闭
	angles_file.close();
	eura_file.close();
}

// 将捕捉到的标记点从图像坐标系转换到相机坐标系
void DataProcess::mapTo3D()
{


	// 双目校正 stereo rectification
	std::vector<cv::Point2f> centerPnt, undistortedPnt;
	for (int camera = 0; camera < numCameras; camera++)
	{
		for (int marker = 0; marker < 6; marker++)
		{
			centerPnt.push_back(points[camera][marker]);
		}
		cv::undistortPoints(centerPnt, undistortedPnt, cameraMatrix[camera], distorCoeff[camera], Rectify[camera], Projection[camera]);
		centerPnt.clear();
		for (int marker = 0; marker < 6; marker++)
		{
			mapped_points[camera][marker] = undistortedPnt[marker];
			/*std::cout << "adding offset: Camera " << camera << " Marker " << marker << " " << points[camera][marker] << "\t";
			std::cout << "float point mapping" << mapped_points[camera][marker] << std::endl;*/
		}
	}	
	
	std::vector<cv::Vec3f> disparityVecLeft, disparityVecRight;

	for (int marker = 0; marker < 6; marker++)
	{
		disparityVecLeft.push_back(cv::Vec3f(mapped_points[0][marker].x,
			mapped_points[0][marker].y, mapped_points[0][marker].y - mapped_points[1][marker].y));
		disparityVecRight.push_back(cv::Vec3f(mapped_points[2][marker].x,
			mapped_points[2][marker].y, mapped_points[2][marker].y - mapped_points[3][marker].y));
	}
	std::vector<cv::Vec3f> MarkerPosVecL, MarkerPosVecR;
	cv::perspectiveTransform(disparityVecLeft, MarkerPosVecL, Q_left);
	cv::perspectiveTransform(disparityVecRight, MarkerPosVecR, Q_right);
	
	for (int marker = 0; marker < 6; marker++)
	{
		MarkerPosVecL[marker] += cv::Vec3f(Transform[0]);
		MarkerPosVecR[marker] += cv::Vec3f(Transform[1]);
		//std::cout << "before move " << marker << " : " << MarkerPosVecL[marker] << std::endl;
	}
	cv::perspectiveTransform(MarkerPosVecL, MarkerPosVecL, Rotation[0]);
	cv::perspectiveTransform(MarkerPosVecR, MarkerPosVecR, Rotation[1]);
	for (int marker = 0; marker < 6; marker++)
	{
		MarkerPos3D[0][marker] = cv::Point3f(MarkerPosVecL[marker]);
		MarkerPos3D[1][marker] = cv::Point3f(MarkerPosVecR[marker]);
		//std::cout << "Camera 0, Marker " << marker << " : " << MarkerPos3D[0][marker] << std::endl;
		//std::cout << "Camera 1, Marker " << marker << " : " << MarkerPos3D[1][marker] << std::endl;
	}
}

// 将上下相机看到的两个点从图像坐标系转换到相机坐标系，并返回
cv::Point3f DataProcess::mapTo3D(int camera_set, cv::Point upper_point, cv::Point lower_point)
{
	std::vector<cv::Point2f> temp_u;
	std::vector<cv::Point2f> temp_l;
	std::vector<cv::Point3f> disparityVec, MarkerPosVec;
	temp_u.push_back(cv::Point2f(upper_point + offset[2*camera_set]));
	temp_l.push_back(cv::Point2f(lower_point+ offset[2 * camera_set+1]));
	cv::undistortPoints(temp_u, temp_u, cameraMatrix[2 * camera_set], distorCoeff[2 * camera_set], Rectify[2 * camera_set], Projection[2 * camera_set]);
	cv::undistortPoints(temp_l, temp_l, cameraMatrix[2 * camera_set+1], distorCoeff[2 * camera_set + 1], Rectify[2 * camera_set + 1], Projection[2 * camera_set + 1]);
	disparityVec.push_back(cv::Point3f(temp_u[0].x, temp_u[0].y, temp_u[0].y - temp_l[0].y));
	if (camera_set == 0)
	{
		cv::perspectiveTransform(disparityVec, MarkerPosVec, Q_left);
	}
	else
	{
		cv::perspectiveTransform(disparityVec, MarkerPosVec, Q_right);
	}
	return cv::Point3f(MarkerPosVec[0]);
}

// 获取关节角度
void DataProcess::getJointAngle()
{
	
	// 所有的坐标现在已经转换到自定义坐标系，矢状面是x-z平面， 额状面是y-z平面
	for (int i = 0; i < 2; i++)
	{

		thigh[i] = MarkerPos3D[i][1] - MarkerPos3D[i][0];
		shank[i] = MarkerPos3D[i][3] - MarkerPos3D[i][2];
		foot[i] = MarkerPos3D[i][5] - MarkerPos3D[i][4];
		/*std::cout << "thigh " << i << " " << thigh[i] << std::endl;
		std::cout << "shank " << i << " " << shank[i] << std::endl;
		std::cout << "foot " << i << " " << foot[i] << std::endl;*/
		// 通过这种余角的方式计算，避免角度从0跳动到360附近，而是呈现正负跳动的形式
		eura_angles[3 * i] = cv::fastAtan2(thigh[i].z, thigh[i].x);
		eura_angles[3 * i] = (eura_angles[3 * i] <= 90) ? 90 - eura_angles[3 * i] :-(eura_angles[3 * i]-90);
		eura_angles[3*i+1] = ((acos((thigh[i].x * shank[i].x + thigh[i].z * shank[i].z) / (sqrt(thigh[i].x * thigh[i].x + thigh[i].z * thigh[i].z) * sqrt(shank[i].x * shank[i].x + shank[i].z * shank[i].z)))) / pi) * 180;
		eura_angles[3*i+2] = ((acos((foot[i].x * shank[i].x + foot[i].z * shank[i].z) / (sqrt(foot[i].x * foot[i].x + foot[i].z * foot[i].z) * sqrt(shank[i].x * shank[i].x + shank[i].z * shank[i].z)))) / pi) * 180;
		//std::cout <<"Camera Set: "<<i<< " hip:   " << hip[i] << "   " << "knee:   " << knee[i] << "   " << "ankle:   " << ankle[i] << std::endl;
	}
	
}

// 输出角度信息到文件，通过Ads向PLC发送步态角
bool DataProcess::exportGaitData()
{
	
	mapTo3D(); // 
	joint_angles_pre = joint_angles;
	getJointAngle(); // 约4ms
	// 滤波后存放到joint_angles
	joint_angles[0] = lpf0.update(eura_angles[0]);
	joint_angles[1]= lpf1.update(eura_angles[1]);
	joint_angles[2] = lpf2.update(eura_angles[2]);
	joint_angles[3] = lpf3.update(eura_angles[3]);
	joint_angles[4] = lpf4.update(eura_angles[4]);
	joint_angles[5] = lpf5.update(eura_angles[5]);
	for (int angle = 0; angle < 6; angle++)
	{
		// 输出未经滤波的真实角度
		angles_file << eura_angles[angle] << ",";
		// 输出滤波后的数据
		angles_file << joint_angles[angle] << ",";
		// 更正欧拉角为两帧之间的差值，之前使用了eura_angles做他用，只是为了节省内存
		eura_angles[angle] = joint_angles[angle] - joint_angles_pre[angle];
		// 输出欧拉角度到文件
		eura_file << eura_angles[angle] << ",";
	}

	angles_file << "\n";
	eura_file << "\n";
	//通过句柄向PLC写入数组
	nErr = AdsSyncWriteReq(pAddr, ADSIGRP_SYM_VALBYHND, lHdlVar2, sizeof(eura_angles), eura_angles);
	if (nErr) std::cerr << "Error: AdsSyncReadReq: " << nErr << '\n';

	return true;
}

// 为左右两个相机坐标系找到从相机坐标系到世界坐标系的转换矩阵
bool DataProcess::FindWorldFrame(cv::Mat images[4])
{
	cv::Size boardsize(3, 3);
	cv::Point3f vector_x, vector_y, vector_z, p0, p1,p2;
	
	for (int camera_set = 0; camera_set < numCameras / 2; camera_set++)
	{
		std::vector<cv::Point3f> vectors(3);
		std::vector<cv::Point2f> corners_upper, corners_lower;
		bool found = cv::findChessboardCorners(images[2 * camera_set], boardsize, corners_upper);
		std::cout << "found :" << found << std::endl;
		// found 为false时， findChessboardCorners不再执行
		found = found && cv::findChessboardCorners(images[2 * camera_set + 1], boardsize, corners_lower);
		std::cout << "found :" << found << std::endl;
		if (!found)
		{
			return false;
		}
		sortChessboardCorners(corners_lower);
		/*for (int i = 0; i < corners_lower.size(); i++)
		{
			std::cout << corners_lower[i] << "\t";
		}
		std::cout << std::endl;*/
		sortChessboardCorners(corners_upper);
		cv::drawChessboardCorners(images[camera_set * 2], boardsize, corners_upper, found);
		cv::drawChessboardCorners(images[camera_set * 2+1], boardsize, corners_lower, found);
		p0 = mapTo3D(camera_set, corners_upper[0], corners_lower[0]);
		p1 = mapTo3D(camera_set, corners_upper[2], corners_lower[2]);
		p2 = mapTo3D(camera_set, corners_upper[6], corners_lower[6]);
		vectors[1] = p1 - p0;
		cv::Point3f transform = -p0;
		vectors[2] = p2 - p0;
		vectors[0] = crossing(vectors[2], vectors[1]);
		vectors[2] = crossing(vectors[0], vectors[1]);
		for (int i = 0; i < 3; i++)
		{
			vectors[i] = scale(vectors[i]);
		}
		// 相机坐标系到自定义坐标系的转换矩阵
		cv::Mat rotation = cv::Mat(3, 3, CV_32FC1, vectors.data()); // 应该是转置后求逆的，但是他是正交矩阵所以不需
		//std::cout << vectors[0] << std::endl;
		
		std::cout << "transform matrix:\n" << transform << std::endl;
		for (int row = 0; row < 4; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				if (row < 3 && col < 3)
				{
					Rotation[camera_set](row, col) = rotation.at<float>(row, col);
				}
				else if (row == 3 && col == 3)
				{
					Rotation[camera_set](row, col) = 1;
				}
				else
				{
					Rotation[camera_set](row, col) = 0;
				}
			}
		}
		std::cout << "rotation matrix:\n" << Rotation[camera_set] << std::endl;
		Transform[camera_set] = cv::Point3f(transform);
	}
	/*cv::namedWindow("corners", 0);
	cv::Mat combine, combine1, combine2;
	cv::hconcat(images[2], images[0], combine1);
	cv::hconcat(images[3], images[1], combine2);
	cv::vconcat(combine1, combine2, combine);
	cv::resize(combine, combine, cv::Size(1024, 1024));
	cv::imshow("corners", combine);
	cv::waitKey(0);*/
	GotWorldFrame = true;
	cv::FileStorage fs("FrameDefine.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs<< "R0" << Rotation[0];
		fs<<"R1"<< Rotation[1];
		fs << "T0" << Transform[0];
		fs << "T1" << Transform[1];
	}
	return true;
}


// 计算叉乘(使用opencv的函数而不用自己的以避免意外）
cv::Point3f crossing(cv::Point3f u, cv::Point3f v)
{
	cv::Mat_<float> Mat_u(3, 1), Mat_v(3,1), result;
	Mat_u(0, 0) = u.x; Mat_v(0, 0) = Mat_v(0, 0);
	Mat_u(1, 0) = u.y; Mat_v(1, 0) = v.y;
	Mat_u(2, 0) = u.z; Mat_v(2, 0) = v.z;
	result = Mat_u.cross(Mat_v);
	return cv::Point3f(result(0,0), result(1,0), result(2,0));
	//return cv::Point3f(u.y * v.z - v.y * u.z, u.z * v.x - v.z * u.x, u.x * v.y - u.y * v.x);
}
// 归一化向量
cv::Point3f scale(cv::Point3f u)
{
	float length = std::sqrt(pow(u.x, 2)+ pow(u.y,2)+pow(u.z,2));
	return u/length;
}

// 使棋盘格按照从右下角到左上角的Z字形顺序排列，以确保计算时使用的是同一个点在两个相机里的投影
void sortChessboardCorners(std::vector<cv::Point2f> &corners)
{	

	std::sort(corners.begin(), corners.end(), comparePointY);
	for (unsigned int i = 0; i < 3; i++)
	{
		std::sort(corners.begin() + i*3, corners.begin() + i*3 + 3, comparePointX);
	}
}


bool comparePointY(cv::Point2f pnt0, cv::Point2f pnt1)
{
	return pnt0.y > pnt1.y;
}
bool comparePointX(cv::Point2f pnt0, cv::Point2f pnt1)
{
	return pnt0.x > pnt1.x;
}
