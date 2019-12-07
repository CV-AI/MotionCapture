#ifndef DATA_PROCESS_HEADER
#define DATA_PROCESS_HEADER

#include <fstream>
#include <deque>
//#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <vector>
//TwinCAT需要的两个头文件
#include <Windows.h> // Ads需要Windows.h,但是不敢动ads本身的文件，所以放在这儿
#include "TcAdsDef.h"
#include "TcAdsAPI.h"
#include "DigitalFilters.h"
#include "ConfigParams.hpp"

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
	// 用于存放各个肢体向量的模，以便判断是否跟丢
    std::vector<std::deque<double>> segmentModule;
	std::vector<cv::Vec3f> initialVecLeft;
	std::vector<cv::Vec3f> initialVecRight;
	bool vecDistInited = false;
	std::vector<double> epsilon = { 10,10,10,10,10,10 };
	int lenCache = 5;
	// 当前帧的角度值存放文件
	std::ofstream angles_file;
	// eura angles 即前后两帧之间的角度差
	std::ofstream eura_file;
	// 相机的数目
	
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
	double anglesToADS[7] = { 0,0,0,0,0,0 ,1};
	
	// 当前帧的关节角
	std::vector<double> filtedAngles;
	// 前一阵的关节角
	std::vector<double> filtedAngle_pre;
	// 是否得到了相机坐标系到世界坐标系的转换矩阵
	bool GotWorldFrame;
	// 相机内参数
	cv::Matx33d cameraMatrix[4];
	// 变形参数
	cv::Mat distorCoeff[4];
	// 4×4  disparity-to-depth mapping matrix (see reprojectImageTo3D ).
	cv::Matx44d Q_left, Q_right;
	// 左右两副相机对，下方相机相对上方相机的旋转矩阵
	cv::Mat rotationMatLeft, rotationMatRight;
	// 左右两副相机对，下方相机相对上方相机的位移
	cv::Mat translationMatLeft, translationMatRight;
	cv::Mat Rectify[4], Projection[4]; // will be computed by cv::stereoRectify()
	// 相机坐标系到世界坐标系的旋转矩阵
	cv::Matx44f Rotation[2];
	// 相机坐标系到世界坐标系的位移
	cv::Point3f Transform[2];
	bool AdsOpened;
	long      nErr, nPort;	//定义端口变量
	AmsAddr   Addr;		//定义AMS地址变量
	PAmsAddr  pAddr = &Addr;  	//定义端口地址变量
	unsigned long lHdlVar2;   	//创建句柄
	char szVar2[20] = { "MAIN.VisionAngle" };
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

