#pragma once

#include <opencv2/opencv.hpp>

#include<iostream>

#include<vector>

#include<cmath>

class operation
{
	void ColorTheresholding();

	static int CorlorsChosen[4];//��ɫ���,˽�б��������Թ���

	bool getmomentpointEx=false;
	
	cv::Point speed;


public:
	operation();
	~operation();

	static void Mouse_getColor(int event, int x, int y, int, void*);
	void getContoursAndMoment();
	//void patternMatch();
	
	static cv::Mat image;
	static bool getColors;
	cv::Mat detectWindow;
	cv::Point momentpoints[6];
	cv::Point detectWindowPosition;
	int threshold = 50;


};

