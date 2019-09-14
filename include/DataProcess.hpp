#ifndef DATA_PROCESS_HEADER
#define DATA_PROCESS_HEADER

#include "Tracker.hpp"
#include <fstream>
#include <opencv2/imgproc/types_c.h>
#include <sstream>
#include "moving_average.hpp"
//TwinCAT需要的两个头文件
#include "TcAdsDef.h"
#include "TcAdsAPI.h"

cv::Point3f crossing(cv::Point3f u, cv::Point3f v);
cv::Point3f scale(cv::Point3f u);
cv::Point3f operator*(cv::Mat M, cv::Point3f p);
void sortChessboardCorners(std::vector<cv::Point2f> &corners);
bool comparePointY(cv::Point2f pnt0, cv::Point2f pnt1);
bool comparePointX(cv::Point2f pnt0, cv::Point2f pnt1);
class DataProcess
{

	const double pi = 3.1415926535898;

public:
	DataProcess();
	~DataProcess();
	//double second, millisecond, deltat = 0;
	cv::Point3d thigh[2]; // 0 for left, 1 for right
	cv::Point3d shank[2];
	cv::Point3d foot[2];
	std::ofstream knee_file;
	std::ofstream hip_file;
	std::ofstream ankle_file;
	const int numCameras = 4;
	//void getTime();
	void mapTo3D();
	cv::Point3f mapTo3D(int, cv::Point, cv::Point);
	void getJointAngle();
	bool exportGaitData();
	bool FrameTransform();
	bool DataProcess::FindWorldFrame(cv::Mat[4]);
	cv::Point2i points[4][6];
	cv::Point2f mapped_points[4][6];
	cv::Point3f MarkerPos3D[2][6];

	// create exponential average object
	EMA ema;
	cv::Mat image;
	double time = 0;
	double hip[2]; // 0 for left, 1 for right
	double knee[2];
	double ankle[2];
	double* eura_angles;
	bool GotWorldFrame;
	bool gettime = false;
	cv::Point2i offset[4] = { cv::Point(500, 500), cv::Point(500,200), cv::Point(750,500), cv::Point(800,200) };
	
	cv::Mat cameraMatrix[4];
	cv::Mat distorCoeff[4];
	// 4×4  disparity-to-depth mapping matrix (see reprojectImageTo3D ).
	cv::Matx44d Q_left, Q_right;
	float cx_list[4] = { 1028.86793241869, 1025.25190697668, 1064.17248516588, 1057.75116447966 };
	float cy_list[4] = { 998.503330516640, 992.096216270560, 1010.52956670735, 1009.38127702806 };
	float fx_list[4] = { 1102.37337240957, 1109.92089040731, 1158.42318268710, 1138.43655571807 };
	float fy_list[4] = { 1102.26282215247, 1108.12606534582, 1170.73160163419, 1150.35306687457 };
	cv::Mat rotationMatLeft, rotationMatRight;
	cv::Mat translationMatLeft, translationMatRight;
	cv::Mat Rectify[4], Projection[4]; // will be computed by cv::stereoRectify()
	cv::Matx44f Rotation[2];
	cv::Point3f Transform[2];

	long      nErr, nPort;	//定义端口变量
	AmsAddr   Addr;		//定义AMS地址变量
	PAmsAddr  pAddr = &Addr;  	//定义端口地址变量
	unsigned long lHdlVar2;   	//创建句柄
	char szVar2[20] = { "MAIN.Array1" };
};
#endif // !DATA_PROCESS_HEADER
