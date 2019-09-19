#ifndef DATA_PROCESS_HEADER
#define DATA_PROCESS_HEADER

#include <fstream>
//#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <sstream>
//TwinCAT需要的两个头文件
#include <Windows.h> // Ads需要Windows.h,但是不敢动ads本身的文件，所以放在这儿
#include "TcAdsDef.h"
#include "TcAdsAPI.h"
#include "DigitalFilters.h"

cv::Point3f crossing(cv::Point3f u, cv::Point3f v);
cv::Point3f scale(cv::Point3f u);
void sortChessboardCorners(std::vector<cv::Point2f> &corners);
bool comparePointY(cv::Point2f pnt0, cv::Point2f pnt1);
bool comparePointX(cv::Point2f pnt0, cv::Point2f pnt1);

class DataProcess
{
public:
	DataProcess();
	~DataProcess();
	// 代表各个肢体的向量，之后用于夹角的计算
	cv::Point3d thigh[2]; // 0 for left, 1 for right
	cv::Point3d shank[2];
	cv::Point3d foot[2];
	// 当前帧的角度值存放文件
	std::ofstream angles_file;
	// eura angles 即前后两帧之间的角度差
	std::ofstream eura_file;
	// 相机的数目
	const int numCameras = 4;
	
	void mapTo3D();
	
	cv::Point3f mapTo3D(int, cv::Point, cv::Point);
	void getJointAngle();
	
	bool exportGaitData();
	
	bool DataProcess::FindWorldFrame(cv::Mat[4]);
	// 标记点在全尺寸图像下的图像坐标
	cv::Point2f points[4][6];
	// 经过校正的图像坐标
	cv::Point2f mapped_points[4][6];
	// 世界坐标系坐标
	cv::Point3f MarkerPos3D[2][6];
	// 欧拉角，由于ads只支持指针传递，所以写成数组
	double eura_angles[6] = { 0,0,0,0,0,0 };
	
	// 当前帧的关节角
	std::vector<double> joint_angles;
	// 前一阵的关节角
	std::vector<double> joint_angles_pre;
	// 是否得到了相机坐标系到世界坐标系的转换矩阵
	bool GotWorldFrame;
	// 捕捉窗口相对全尺寸图像的偏置
	cv::Point2i offset[4] = { cv::Point(500, 500), cv::Point(500,200), cv::Point(550,500), cv::Point(600,200) };
	// 相机内参数
	cv::Matx33d cameraMatrix[4];
	// 变形参数
	cv::Mat distorCoeff[4];
	// 4×4  disparity-to-depth mapping matrix (see reprojectImageTo3D ).
	cv::Matx44d Q_left, Q_right;
	// 光心
	float cx_list[4] = { 1028.86793241869, 1025.25190697668, 1064.17248516588, 1057.75116447966 };
	float cy_list[4] = { 998.503330516640, 992.096216270560, 1010.52956670735, 1009.38127702806 };
	// 焦距
	float fx_list[4] = { 1102.37337240957, 1109.92089040731, 1158.42318268710, 1138.43655571807 };
	float fy_list[4] = { 1102.26282215247, 1108.12606534582, 1170.73160163419, 1150.35306687457 };
	// 左右两副相机对，下方相机相对上方相机的旋转矩阵
	cv::Mat rotationMatLeft, rotationMatRight;
	// 左右两副相机对，下方相机相对上方相机的位移
	cv::Mat translationMatLeft, translationMatRight;
	cv::Mat Rectify[4], Projection[4]; // will be computed by cv::stereoRectify()
	// 相机坐标系到世界坐标系的旋转矩阵
	cv::Matx44f Rotation[2];
	// 相机坐标系到世界坐标系的位移
	cv::Point3f Transform[2];
	long      nErr, nPort;	//定义端口变量
	AmsAddr   Addr;		//定义AMS地址变量
	PAmsAddr  pAddr = &Addr;  	//定义端口地址变量
	unsigned long lHdlVar2;   	//创建句柄
	char szVar2[20] = { "MAIN.Array1" };
};
constexpr float dtUsed = 0.025;
constexpr double tau = 2; // 滤波器的时间常数
// Construct various filter with cutoff frequency of 0.5 Hz.
// 无论是写成vector还是[]，最终会因为Filter对象没有被初始化而出错
// 只有这种方式能初始化对象（可能是因为Filter的作者没有实现其他的初始化方式）
static LowPassFilter3 lpf0(dtUsed, tau);
static LowPassFilter3 lpf1(dtUsed, tau);
static LowPassFilter3 lpf2(dtUsed, tau);
static LowPassFilter3 lpf3(dtUsed, tau);
static LowPassFilter3 lpf4(dtUsed, tau);
static LowPassFilter3 lpf5(dtUsed, tau);
#endif // !DATA_PROCESS_HEADER

