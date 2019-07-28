#include "Tracker.hpp"

class DataProcess
{
	bool gettime = false;
	double second, millisecond, deltat = 0;

	cv::Point2d thigh, shank, foot;
	const double pi = 3.1415926535898;

public:
	DataProcess();
	~DataProcess();
	int numCameras;
	//void getTime();
	void mapTo3D();
	void getJointAngle();
	cv::Point points[4][6];
	cv::Point3d MarkerPos3D[2][6];
	bool exportGaitData();
	cv::Mat image;
	double time = 0;
	double hip, knee, ankle;
};

