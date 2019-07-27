#pragma once

#include <opencv2/opencv.hpp>
// #include <opencv2/gpu/gpu.hpp>
#include<iostream>

#include<vector>

#include<cmath>

#define MAX_H_RED 240
#define MIN_H_RED 300
class Tracker
{
	void ColorTheresholding();

	static int CorlorsChosen[4];

	bool getmomentpointEx=false;
	
	cv::Point speed;
public:
	Tracker();
	~Tracker();
	// enable different tracker 
	enum TrackerType{ByDetection, CV_KCF};
	static void Mouse_getColor(int event, int x, int y, int, void*);
	bool getContoursAndMoment(int camera_index);
	//void patternMatch();
	
	static cv::Mat image;
	static bool getColors;
    cv::Mat ReceivedImages[4]; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	cv::Mat detectWindow;
	cv::Point momentpoints[6];
	cv::Point detectWindowPosition = cv::Point(0, 0);
	int threshold = 70;
	cv::Point currentPos[4][6]; // first entry is the index of image, second entry is the index of marker
	cv::Point previousPos[4][6];
	cv::Point predictPos; // the predicted position of marker in current frame
	int detectWindowDimX = 50; // the dimension of detectWindow
	int detectWindowDimY = 50; 
	bool InitTracker(TrackerType);
	bool TrackerIntialized = false;
	bool UpdateTracker(TrackerType);
	bool RectifyMarkerPos(int);
};

