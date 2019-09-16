#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include<iostream>
#include <opencv2/highgui/highgui_c.h>
#include<string>
#include<vector>
#include <Windows.h>
#include<cmath>

// MAX and MIN H, S, V
const int MAX_H_RED = 50;
const int  MIN_H_RED = 0;
const int MAX_SATURATION = 255;
const int MIN_SATURATION = 120;
const int MAX_VALUE = 255;
const int MIN_VALUE = 120;
const int NUM_CAMERAS = 4;
const int NUM_MARKERS = 6;
const int NUM_MARKER_SET = 3;
const double weight = 0.75;
enum TrackerType { ByDetection, CV_KCF, ByColor };

class Tracker
{
	
public:
	Tracker();
	~Tracker();

	void ColorThresholding(int);
	void ColorThresholding();
	static cv::Scalar CorlorsChosen;

	// enable different tracker 
	
	static void Mouse_getColor(int event, int x, int y, int, void*);
	static void Mouse_getRegion(int event, int x, int y, int, void*);
	static cv::Rect calibration_region;
	bool initMarkerPosition(int camera_index);
	bool updateMarkerPosition(int camera_index, int marker_set);
	//void patternMatch();
	
	static cv::Mat image;
	static bool getColors;
    static cv::Mat ReceivedImages[NUM_CAMERAS]; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	cv::Mat detectWindow;
	cv::Mat detectWindow_Initial;
	cv::Point detectPosition;

	static cv::Point currentPos[NUM_CAMERAS][NUM_MARKERS]; // first entry is the index of image, second entry is the index of marker
	static cv::Point previousPos[NUM_CAMERAS][NUM_MARKERS];// make it static to share between multiple tracker objects
	static cv::Point currentPosSet[NUM_CAMERAS][NUM_MARKER_SET]; // current position of camera set
	static cv::Point previousPosSet[NUM_CAMERAS][NUM_MARKER_SET];
	cv::Point momentum[NUM_MARKER_SET]; // 动量：即前两帧的位置差
	
	//int cmin = 80; // minimum and maximum value for contours
	//int cmax = 140;
	bool InitTracker(TrackerType);
	bool FilterInitialImage();
	bool RectifyMarkerPos(int);
	int detectWindowDimX; // the dimension of detectWindow_Initial
	int detectWindowDimY;
	int threshold;
	bool TrackerAutoIntialized;
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

DWORD WINAPI UpdateTracker(LPVOID lpParam);


