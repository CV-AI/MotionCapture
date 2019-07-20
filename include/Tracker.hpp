#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include<iostream>

#include<vector>

#include<cmath>

class Tracker
{
	void ColorTheresholding();

	static int CorlorsChosen[4];

	bool getmomentpointEx=false;
	
	cv::Point speed;


public:
	Tracker();
	~Tracker();

	static void Mouse_getColor(int event, int x, int y, int, void*);
	bool getContoursAndMoment(int camera_index);
	//void patternMatch();
	
	static cv::Mat image;
	static bool getColors;
    cv::Mat ReceivedImages[4]; // Left_Upper, Right_Upper, Right_Lower, Left_Lower
	cv::Mat detectWindow;
	cv::Point momentpoints[6];
	cv::Point detectWindowPosition;
	int threshold = 50;
	cv::Point currentPos[4][6]; // first entry is the index of image, second entry is the index of marker
	cv::Point previousPos[4][6];
	bool InitTracker();
	bool TrackerIntialized = false;
	bool UpdateTracker();
	bool RectifyMarkerPos(int);

};

