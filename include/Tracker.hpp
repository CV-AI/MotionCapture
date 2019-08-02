#pragma once

#include <opencv2/opencv.hpp>
// #include <opencv2/gpu/gpu.hpp>
#include<iostream>
#include<string>
#include<vector>
#include <Windows.h>
#include<cmath>

constexpr auto MAX_H_RED = 40;
constexpr auto MIN_H_RED = 0;
const int NUM_CAMERAS = 4;
const int NUM_MARKERS = 6;
enum TrackerType { ByDetection, CV_KCF, ByColor };

class Tracker
{
	
public:
	Tracker();
	~Tracker();
	void ColorThresholding(int);
	void ColorThresholding();
	static int CorlorsChosen[3];
	cv::Point speed;
	// enable different tracker 
	
	static void Mouse_getColor(int event, int x, int y, int, void*);
	static void Mouse_getRegion(int event, int x, int y, int, void*);
	static cv::Rect calibration_region;
	bool getContoursAndMoment(int camera_index);
	bool getContoursAndMoment(int camera_index, int marker_index);
	//void patternMatch();
	
	static cv::Mat image;
	static bool getColors;
    static cv::Mat ReceivedImages[NUM_CAMERAS]; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	cv::Mat detectWindow;
	cv::Mat detectWindow_Initial;
	cv::Point detectPosition;
	cv::Point detectPosition_Initial;
	static cv::Point currentPos[NUM_CAMERAS][NUM_MARKERS]; // first entry is the index of image, second entry is the index of marker
	static cv::Point previousPos[NUM_CAMERAS][NUM_MARKERS];// make it static to share between multiple tracker object
	cv::Point predictPos; // the predicted position of marker in current frame
	
	//int cmin = 80; // minimum and maximum value for contours
	//int cmax = 140;
	bool InitTracker(TrackerType);
	bool FilterInitialImage();
	bool RectifyMarkerPos(int);
	int detectWindowDimX; // the dimension of detectWindow_Initial
	int detectWindowDimY;
	int numCameras;
	int threshold;
	bool TrackerAutoIntialized;
};
class TrackerParameters
{
public:
	int marker_index;
	TrackerType tracker_type;
	Tracker* trackerPtr;
	TrackerParameters()
	{
		marker_index = 0;
		trackerPtr = NULL;
		tracker_type = ByColor;
	}
	~TrackerParameters()
	{

	}
};

#if defined (_WIN32)
DWORD WINAPI UpdateTracker(LPVOID lpParam);
#endif

