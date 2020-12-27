#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include <opencv2/highgui/highgui_c.h>
#include <string>
#include <vector>
#include <Windows.h>
#include <cmath>
#include "ConfigParams.hpp"

enum TrackerType { ByDetection, CV_KCF, ByColor };
const int DETECT_WINDOW_BOUNDRY = 20;
class Tracker
{
	
public:
	Tracker();
	~Tracker();
	
	static cv::Rect calibration_region;
	bool manualInitMarkerPosition(int camera_index);
	bool autoInitMarkerPosition(int camera_index);
	bool updateMarkerPosition(int camera_index, int marker_set);
	//void patternMatch();
	// 类中的静态变量在各个对象中共用
	static cv::Mat image;
	static bool getColors;
    static cv::Mat ReceivedImages[NUM_CAMERAS]; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	cv::Mat detectWindow;
	cv::Mat detectWindow_Initial;
	cv::Mat mask_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::Point detectPosition;
	// 目前帧和上一帧中检测到的标记点的位置
	static cv::Point2f currentPos[NUM_CAMERAS][NUM_MARKERS]; // first entry is the index of image, second entry is the index of marker
	static cv::Point2f previousPos[NUM_CAMERAS][NUM_MARKERS];// make it static to share between multiple tracker objects
	// 目前帧和上一帧中检测到的标记点对的位置
	static cv::Point2f currentPosSet[NUM_CAMERAS][NUM_MARKER_SET]; // current position of camera set
	static cv::Point2f previousPosSet[NUM_CAMERAS][NUM_MARKER_SET];
	// 动量：即前两帧的位置差
	cv::Point2f momentum[NUM_MARKER_SET]; 
	
	bool InitTracker(TrackerType); 
	bool RectifyMarkerPos(int);
	// 每个检测窗口的大小
	static cv::Point2i detectWindowDim[NUM_CAMERAS][NUM_MARKER_SET];
	static cv::Point2i detectWindowDimMin[NUM_CAMERAS][NUM_MARKER_SET];
	int threshold;
	bool TrackerAutoIntialized;
	void updateDetectWindowDims();
}; 
class TrackerParameters
{
public:
	int camera_index;
	TrackerType tracker_type;
	Tracker* trackerPtr;
	TrackerParameters()
	{
		camera_index = 0;
		trackerPtr = nullptr;
		tracker_type = ByColor;
	}
	~TrackerParameters()
	{

	}
};
float pointDist(const cv::Point2f& p0, const cv::Point2f& p1);
int encodeErrorMarkerSets(std::vector<int>);
cv::Mat colorThresholding(const cv::Mat& color_img,bool tracking);
DWORD WINAPI UpdateTracker(LPVOID lpParam);


