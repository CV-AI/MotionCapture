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

cv::Point3d crossing(cv::Point3d u, cv::Point3d v);
cv::Point3d scale(cv::Point3d u);
cv::Point3d operator*(cv::Mat M, cv::Point3d p);
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
	int numCameras;
	//void getTime();
	void mapTo3D();
	void getJointAngle();
	bool exportGaitData();
	bool FrameTransform();
	bool DataProcess::FindWorldFrame(cv::Mat, cv::Mat);
	cv::Point points[4][6];
	cv::Point3d MarkerPos3D[2][6];
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
	const double cx = 1010.13238776137;
	const double cy = 991.338656997527;
	const double fx = 1121.56188766987;
	const double fy = 1120.60483176776;
	const double cy_lower = 997.561141920296;
	cv::Mat cameraMatrix[4];
	cv::Mat distorCoeff[4];
	// 4×4  disparity-to-depth mapping matrix (see reprojectImageTo3D ).
	cv::Matx44d Q_left, Q_right;
	float cx_list[4] = { 1010.13238776137, 1010.13238776137 ,1010.13238776137 ,1010.13238776137 };
	float cy_list[4] = { 991.338656997527, 991.338656997527,991.338656997527,991.338656997527 };
	float fx_list[4] = { 1121.56188766987, 1121.56188766987,1121.56188766987,1121.56188766987 };
	float fy_list[4] = { 1120.60483176776, 1120.60483176776,1120.60483176776,1120.60483176776 };
	cv::Mat rotationMatLeft, rotationMatRight;
	cv::Mat translationMatLeft, translationMatRight;
	cv::Mat mapX[4];
	cv::Mat mapY[4];
	const double delta_cy = 6.222484902473; // cy_lower - cy
	const double T = 244.628114017665;
	std::vector<cv::Mat> Rotation;
	std::vector<cv::Point3d> Transform;

	long      nErr, nPort;	//定义端口变量
	AmsAddr   Addr;		//定义AMS地址变量
	PAmsAddr  pAddr = &Addr;  	//定义端口地址变量
	unsigned long lHdlVar2;   	//创建句柄
	char szVar2[20] = { "MAIN.Array1" };
};
#endif // !DATA_PROCESS_HEADER

