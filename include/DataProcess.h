#include "Tracker.hpp"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/imgproc/types_c.h>
class DataProcess
{
	
	const double pi = 3.1415926535898;

public:
	DataProcess();
	~DataProcess();
	double second, millisecond, deltat = 0;
	cv::Point3d thigh[2]; // 0 for left, 1 for right
	cv::Point3d shank[2];
	cv::Point3d foot[2];
	int numCameras;
	//void getTime();
	void mapTo3D();
	void getJointAngle();
	bool exportGaitData();
	bool FrameTransform();
	bool DataProcess::FindWorldFrame(cv::Mat,cv::Mat);
	cv::Point points[4][6];
	cv::Point3d MarkerPos3D[2][6];

	cv::Mat image;
	double time = 0;
	double hip[2]; // 0 for left, 1 for right
	double knee[2];
	double ankle[2];
	bool GotWorldFrame;
	bool gettime = false;
};

