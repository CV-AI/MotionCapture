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
#include "ConfigParams.hpp"

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
	cv::Mat mask_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	cv::Point detectPosition;

	static cv::Point2f currentPos[NUM_CAMERAS][NUM_MARKERS]; // first entry is the index of image, second entry is the index of marker
	static cv::Point2f previousPos[NUM_CAMERAS][NUM_MARKERS];// make it static to share between multiple tracker objects
	static cv::Point2f currentPosSet[NUM_CAMERAS][NUM_MARKER_SET]; // current position of camera set
	static cv::Point2f previousPosSet[NUM_CAMERAS][NUM_MARKER_SET];
	cv::Point2f momentum[NUM_MARKER_SET]; // 动量：即前两帧的位置差

	bool InitTracker(TrackerType); 
	bool FilterInitialImage();
	bool RectifyMarkerPos(int);
	const cv::Point2i detectWindowDim[NUM_CAMERAS][NUM_MARKER_SET] = { 
										{{110, 150}, {110, 110}, {130, 100} },
										{{110, 150}, {110, 110}, {130, 100} },
										{{110, 150}, {110, 110}, {120, 100} },
										{{110, 150}, {110, 110}, {120, 100} } };
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


