#pragma once

#include <opencv2/opencv.hpp>
// #include <opencv2/gpu/gpu.hpp>
#include<iostream>
#include<string>
#include<vector>

#include<cmath>

constexpr auto MAX_H_RED = 40;
constexpr auto MIN_H_RED = 0;
const int NUM_CAMERAS = 4;
const int NUM_MARKERS = 6;
class TrackerParameters
{
public:
	TrackerParameters()
	{

	}
}
class Tracker
{
	void ColorTheresholding();

	static int CorlorsChosen[3];

	bool getmomentpointEx=false;
	
	cv::Point speed;
public:
	Tracker();
	~Tracker();
	// enable different tracker 
	enum TrackerType{ByDetection, CV_KCF, };
	static void Mouse_getColor(int event, int x, int y, int, void*);
	static void Mouse_getRegion(int event, int x, int y, int, void*);
	static cv::Rect calibration_region;
	bool getContoursAndMoment(int camera_index);
	bool getContoursAndMoment(int camera_index, int marker_index);
	//void patternMatch();
	
	static cv::Mat image;
	static bool getColors;
    cv::Mat ReceivedImages[NUM_CAMERAS]; // Left_Upper, qRight_Upper, Right_Lower, Left_Lower
	cv::Mat detectWindow;
	cv::Point momentpoints[NUM_MARKERS];
	cv::Point detectWindowPosition = cv::Point(0, 0);
	int threshold = 100;
	cv::Point currentPos[NUM_CAMERAS][NUM_MARKERS]; // first entry is the index of image, second entry is the index of marker
	cv::Point previousPos[NUM_CAMERAS][NUM_MARKERS];
	cv::Point predictPos; // the predicted position of marker in current frame
	int detectWindowDimX = 80; // the dimension of detectWindow
	int detectWindowDimY = 150; 
	int numCameras = 0;
	int cmin = 80; // minimum and maximum value for contours
	int cmax = 140;
	bool InitTracker(TrackerType);
	bool FilterInitalImage();
	bool TrackerIntialized = false;
	bool UpdateTracker(TrackerType, int);
	bool RectifyMarkerPos(int);
};