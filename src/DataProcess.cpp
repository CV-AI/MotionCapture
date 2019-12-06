#include "DataProcess.hpp"
#include <algorithm>
#include <iostream>

DataProcess::DataProcess() :GotWorldFrame(false)
{
	GotWorldFrame = false;
	AdsOpened = false;
	filtedAngles = { 0,0,0,0,0,0 };
	filtedAngle_pre = { 0,0,0,0,0,0 };
	
	angles_file.open("angles.csv");
	eura_file.open("eura.csv");
	cv::FileStorage fs_calib("calib_params.yml", cv::FileStorage::READ);
	if (fs_calib.isOpened())
	{
		for (int camera = 0; camera < NUM_CAMERAS; camera++)
		{
			
			// IntrinsicMatrix generated in matlab must be transposed to use in opencv（data in yml file is already transposed)
			char* str = new char[strlen("cameraMatrix")+1];
			sprintf(str, "%s%d", "cameraMatrix", camera);
			fs_calib[str] >> cameraMatrix[camera];
			// OpenCV's distortion coefficient vector:  the radial parameters come first; these are followed by the
			// two tangential parameters --LearnOpenCV3 Chapter19 Page724
			// this problem is handled in matlab script "save_camera_calib_to_yml.m"
			str = new char[strlen("Distortion") + 1];
			sprintf(str, "%s%d", "Distortion", camera);
			fs_calib[str] >> distorCoeff[camera];
			// use CV_64F type for inputs of function of stereoRectify
			distorCoeff[camera].convertTo(distorCoeff[camera], CV_64F);
			std::cout << "Camera " << camera << " Matrix: \n" << cameraMatrix[camera] << std::endl;
			std::cout << "Camera " << camera << " distortion vector: \n" << distorCoeff[camera] << std::endl;
		}
		// initialize rotation and translation matrix for left and right camera set
		// the matlab matrix need to be transposed to fit opencv
		// use CV_64F type for inputs of function of stereoRectify
		
		fs_calib["Rotation0"] >> rotationMatLeft;
		fs_calib["Rotation1"] >> rotationMatRight;
		rotationMatLeft.convertTo(rotationMatLeft, CV_64F);
		rotationMatRight.convertTo(rotationMatRight, CV_64F);
		std::cout << "Left Camera Set rotation Matrix: \n" << rotationMatLeft << std::endl;
		std::cout << "Right Camera Set rotation Matrix: \n" << rotationMatRight << std::endl;
		// use CV_64F type for inputs of function of stereoRectify
		translationMatLeft = cv::Mat(3, 1, CV_64F);
		translationMatRight = cv::Mat(3, 1, CV_64F);
		fs_calib["Translation0"] >> translationMatLeft;
		fs_calib["Translation1"] >> translationMatRight;
		translationMatLeft.convertTo(translationMatLeft, CV_64F);
		translationMatRight.convertTo(translationMatRight, CV_64F);
		std::cout << "Left Camera Set translation Matrix: \n" << translationMatLeft << std::endl;
		std::cout << "Right Camera Set translation Matrix: \n" << translationMatRight << std::endl;
	}
	
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
	for (int camera = 0; camera < NUM_CAMERAS; camera++)
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
		}
		if (_PRINT_PROCESS)
		{
			for (int marker = 0; marker < 6; marker++)
			{
				std::cout << "Marker Pos after adding offset " << camera << " marker " << marker << " " << points[camera][marker] << std::endl;
				std::cout << "After undistort mapping" << mapped_points[camera][marker] << std::endl;
			}
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
	}
	if (_PRINT_PROCESS)
	{
		for (int marker = 0; marker < NUM_MARKERS; marker++)
		{
			std::cout << "3D Pos before frame change:Left  " << marker << " : " << MarkerPosVecL[marker] << std::endl;
			std::cout << "3D Pos before frame change:Right " << marker << " : " << MarkerPosVecR[marker] << std::endl;
		}
	}
	cv::perspectiveTransform(MarkerPosVecL, MarkerPosVecL, Rotation[0]);
	cv::perspectiveTransform(MarkerPosVecR, MarkerPosVecR, Rotation[1]);
	for (int marker = 0; marker < NUM_MARKERS; marker++)
	{
		MarkerPos3D[0][marker] = cv::Point3f(MarkerPosVecL[marker]);
		MarkerPos3D[1][marker] = cv::Point3f(MarkerPosVecR[marker]);
		if (_PRINT_PROCESS)
		{
			std::cout << "3D Pos of Camera 0, Marker " << marker << " : " << MarkerPos3D[0][marker] << std::endl;
			std::cout << "3D Pos of Camera 1, Marker " << marker << " : " << MarkerPos3D[1][marker] << std::endl;
		}
	}
	if (vecDistInited)
	{
		for (size_t segment = 0; segment < NUM_MARKERS / 2; segment++)
		{
			segmentModule[segment].push_back(cv::norm(MarkerPosVecL[segment] - MarkerPosVecL[segment + 3]
									- initialVecLeft[segment]));
			segmentModule[segment + 3].push_back(cv::norm(MarkerPosVecR[segment] - MarkerPosVecR[segment + 3]
									- initialVecRight[segment]));
		}
		if (segmentModule[0].size() > lenCache)
		{
			for (size_t segment = 0; segment < NUM_MARKERS / 2; segment++)
			{
				segmentModule[segment].pop_front();
				segmentModule[segment + 3].pop_front();
			}
		}
	}
	else
	{
		for (size_t segment = 0; segment < NUM_MARKERS / 2; segment++)
		{
			segmentModule.push_back(std::deque<double>(cv::norm(MarkerPosVecL[segment] - MarkerPosVecL[segment + 3])));
			segmentModule.push_back(std::deque<double>(cv::norm(MarkerPosVecL[segment] - MarkerPosVecL[segment + 3])));
			initialVecLeft.push_back(MarkerPosVecL[segment] - MarkerPosVecL[segment + 3]);
			initialVecRight.push_back(MarkerPosVecR[segment] - MarkerPosVecL[segment + 3]);
		}
		vecDistInited = true;
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
		anglesToADS[3 * i] = cv::fastAtan2(thigh[i].z, thigh[i].x);
		anglesToADS[3 * i] = (anglesToADS[3 * i] <= 90) ? 90 - anglesToADS[3 * i] :-(anglesToADS[3 * i]-90);
		anglesToADS[3*i+1] = ((acos((thigh[i].x * shank[i].x + thigh[i].z * shank[i].z) / (sqrt(thigh[i].x * thigh[i].x + thigh[i].z * thigh[i].z) * sqrt(shank[i].x * shank[i].x + shank[i].z * shank[i].z)))) / pi) * 180;
		anglesToADS[3*i+2] = ((acos((foot[i].x * shank[i].x + foot[i].z * shank[i].z) / (sqrt(foot[i].x * foot[i].x + foot[i].z * foot[i].z) * sqrt(shank[i].x * shank[i].x + shank[i].z * shank[i].z)))) / pi) * 180;
	}
	
}

// 输出角度信息到文件，通过Ads向PLC发送步态角
bool DataProcess::exportGaitData()
{
	
	mapTo3D(); 
	filtedAngle_pre = filtedAngles;
	getJointAngle(); 
	// 滤波后存放到joint_angles
	filtedAngles[0] = lpf0.update(anglesToADS[0]);
	filtedAngles[1]= lpf1.update(anglesToADS[1]);
	filtedAngles[2] = lpf2.update(anglesToADS[2]);
	filtedAngles[3] = lpf3.update(anglesToADS[3]);
	filtedAngles[4] = lpf4.update(anglesToADS[4]);
	filtedAngles[5] = lpf5.update(anglesToADS[5]);
	for (int angle = 0; angle < 6; angle++)
	{
		// 输出未经滤波的真实角度
		angles_file << anglesToADS[angle] << ",";
		// 输出滤波后的数据
		angles_file << filtedAngles[angle] << ",";
		// 更正欧拉角为两帧之间的差值，之前使用了eura_angles做他用，只是为了节省内存
		anglesToADS[angle] = filtedAngles[angle] - filtedAngle_pre[angle];
		// 输出欧拉角度到文件
		eura_file << anglesToADS[angle] << ",";
		// 注意最终输出给ads的其实是真实值
		anglesToADS[angle] = filtedAngles[angle];
	}
	// 将左右腿的数值交换数值，以供控制系统使用
	for (int angle = 0; angle < 3; angle++)
	{
		std::swap(anglesToADS[angle], anglesToADS[angle + 3]);
	}
	angles_file << "\n";
	eura_file << "\n";
	if (AdsOpened) // 必须作此判断，否则Ads会消耗近5秒的时间，应该是在重试
	{
		//通过句柄向PLC写入数组
		nErr = AdsSyncWriteReq(pAddr, ADSIGRP_SYM_VALBYHND, lHdlVar2, sizeof(anglesToADS), anglesToADS);
		if (nErr) std::cerr << "Error: AdsSyncReadReq: " << nErr << '\n';
	}
	return true;
}

// 为左右两个相机坐标系找到从相机坐标系到世界坐标系的转换矩阵
bool DataProcess::FindWorldFrame(cv::Mat images[4])
{
	cv::Size boardsize(3, 3);
	cv::Point3f vector_x, vector_y, vector_z, p0, p1,p2;
	
	for (int camera_set = 0; camera_set < NUM_CAMERAS / 2; camera_set++)
	{
		std::vector<cv::Point3f> vectors(3);
		std::vector<cv::Point2f> corners_upper, corners_lower;
		bool found = cv::findChessboardCorners(images[2 * camera_set], boardsize, corners_upper);
		// found 为false时， findChessboardCorners不再执行
		found = found && cv::findChessboardCorners(images[2 * camera_set + 1], boardsize, corners_lower);
		if (!found) return false;
		
		sortChessboardCorners(corners_lower);
		/*for (int i = 0; i < corners_lower.size(); i++)
		{
			std::cout << corners_lower[i] << "\t";
		}
		std::cout << std::endl;*/
		sortChessboardCorners(corners_upper);
		cv::drawChessboardCorners(images[camera_set * 2], boardsize, corners_upper, found);
		cv::drawChessboardCorners(images[camera_set * 2+1], boardsize, corners_lower, found);
		cv::namedWindow("corners", 0);
		cv::Mat combine, combine1, combine2;
		cv::hconcat(images[2], images[0], combine1);
		cv::hconcat(images[3], images[1], combine2);
		cv::vconcat(combine1, combine2, combine);
		cv::resize(combine, combine, cv::Size(1024, 1024));
		cv::imshow("corners", combine);
		cv::waitKey(0);
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
