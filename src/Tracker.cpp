// 此文件用于初始化跟踪 
#include "Tracker.hpp"
#include <algorithm>


Tracker::Tracker()
{
}


Tracker::~Tracker()
{
}
cv::Mat Tracker:: image;
int Tracker::CorlorsChosen[3];
bool Tracker::getColors = false;
bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	double i = fabs(cv::contourArea(cv::Mat(contour1)));
	double j = fabs(cv::contourArea(cv::Mat(contour2)));
	return (i < j);
}

// initialize with whole image, so detectWindowPosition is (0, 0)
// This function get the color of the marker
void Tracker::Mouse_getColor(int event, int x, int y, int, void*)
{
	static cv::Point origin;
	static cv::Rect selection;
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = cv::Point(x, y);
		break;
	case CV_EVENT_LBUTTONUP:
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = abs(x - origin.x);
		selection.height = abs(y - origin.y);
		std::cout << "Color area has been selected!" << std::endl;
		std::cout << "Color has been selected" << std::endl;
		image = image(selection);
		cv::Scalar meanScalar = cv::mean(image);
		CorlorsChosen[0] = static_cast<int>(meanScalar.val[0]);
		CorlorsChosen[1] = static_cast<int>(meanScalar.val[1]);
		CorlorsChosen[2] = static_cast<int>(meanScalar.val[2]);
		getColors = true;
		std::cout << "blue green red: " << CorlorsChosen[0] << " " << CorlorsChosen[1] << " " << CorlorsChosen[2] << std::endl;
	}
}



void Tracker::ColorTheresholding()
{
	cv::cvtColor(detectWindow, detectWindow, CV_RGB2HSV);
	cv::Mat rangeRes = cv::Mat::zeros(detectWindow.size(), CV_8UC1);
	cv::inRange(detectWindow, cv::Scalar(MIN_H_RED, 100, 80), cv::Scalar(MAX_H_RED, 255, 255), rangeRes);
	detectWindow = rangeRes;
	/*for (int j = 0; j < detectWindow.rows; j++)
	{
		uchar*data = detectWindow.ptr<uchar>(j);
		
		for (int i = 0; i< detectWindow.cols; i++)
		{
			
			if ((abs(data[3 * i] - CorlorsChosen[0]) + abs(data[3 * i + 1] - CorlorsChosen[1]) +
					abs(data[3 * i + 2] - CorlorsChosen[2]))< threshold)
			{
				data[3 * i] = data[3 * i + 1] = data[3 * i + 2] = 255;
			}
			else data[3 * i] = data[3 * i + 1] = data[3 * i + 2] = 0;
		}
	}*/
}

// This function using contours to get all of six marker positions for one camera
bool Tracker:: getContoursAndMoment(int camera_index)
{	
	ColorTheresholding();
	
	//cv::fastNlMeansDenoising(detectWindow, detectWindow);
	//cv::Mat detectCopy = detectWindow.clone();
	//cv::cvtColor(detectWindow, detectWindow, CV_BGRA2GRAY);
	cv::Mat mask(9, 9, CV_8U, cv::Scalar(1));
	cv::morphologyEx(detectWindow, detectWindow, cv::MORPH_CLOSE, mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(detectWindow, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
	cv::drawContours(ReceivedImages[camera_index], contours, -1, cv::Scalar(150, 100, 0), 2);//-1 means draw all contours
	// remove contours that are too small or large
	while (itc != contours.end())
	{
		std::cout << itc->size() << std::endl;
		//std::cout << "size: " << itc->size() << std::endl;
		if (itc->size() < cmin || itc->size() > cmax)
		{
			itc = contours.erase(itc);
		}
		else itc++;
	}
	if (camera_index == 3)
	{
		std::cout << "hi" << std::endl;
		cv::namedWindow("detectwindow", 0);
		cv::imshow("detectwindow", detectWindow);
		cv::waitKey(1);
	}
	/*std::sort(contours.begin(), contours.end(), compareContourAreas);
	std::vector<std::vector<cv::Point>>sliced_contours = std::vector<std::vector<cv::Point>>(contours.begin(), contours.begin() + 6);*/
	/*cv::namedWindow("detectwindow", 0);
	cv::imshow("detectwindow", detectWindow);
	cv::waitKey(1);*/
	if (contours.size()==6)
	{
		std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();

		int i = 0;
		while (it != contours.end())
		{

			cv::Moments mom = cv::moments(cv::Mat(*it++));
			// 因为检测窗口不是全部画面，所以要加上检测窗口在全部画面的位置
			currentPos[camera_index][i] = cv::Point(mom.m10 / mom.m00+detectWindowPosition.x, mom.m01 / mom.m00+ detectWindowPosition.y);
			i++;
		}
		cv::drawContours(ReceivedImages[camera_index], contours,-1, cv::Scalar(0, 255, 0), 5);//-1 means draw all contours
		return true;
	}
	else
	{
		// TODO: 如果使用颜色跟踪失败则需要使用其他跟踪方法
		std::cout << "Contours numbers are wrong:  "<< contours.size()<<std::endl;
		return false;
	}
}

// This function get the marker point for specific marker in a specific camera
bool Tracker::getContoursAndMoment(int camera_index, int marker_index)
{
	ColorTheresholding();
	//cv::Mat detectCopy = detectWindow.clone();
	//cv::cvtColor(detectWindow, detectWindow, CV_BGRA2GRAY);
	cv::Mat mask(5, 5, CV_8U, cv::Scalar(1));
	cv::morphologyEx(detectWindow, detectWindow, cv::MORPH_CLOSE, mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(detectWindow, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
	// remove contours that are too small or large
	while (itc != contours.end())
	{
		std::cout << itc->size() << std::endl;
		//std::cout << "size: " << itc->size() << std::endl;
		if (itc->size() < cmin || itc->size() > cmax)
		{
			itc = contours.erase(itc);
		}
		else itc++;
	}
	/*cv::namedWindow("detectwindow", 0);
	cv::imshow("detectwindow", detectCopy);
	cv::waitKey(1);*/
	if (contours.size() == 1)
	{
		std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();

		cv::Moments mom = cv::moments(cv::Mat(*it++));
		// 因为检测窗口不是全部画面，所以要加上检测窗口在全部画面的位置
		currentPos[camera_index][marker_index] = cv::Point(mom.m10 / mom.m00 + detectWindowPosition.x, mom.m01 / mom.m00 + detectWindowPosition.y);

		return true;
	}
	else
	{
		// TODO: 如果使用颜色跟踪失败则需要使用其他跟踪方法
		std::cout << "Contours numbers are wrong:  " << contours.size() << std::endl;
		return false;
	}
}

// 为了方便使用，初始化tracker时都应该实际使用ByDetection（使用其他的tracker都需要自行画出框图）
bool Tracker::InitTracker(TrackerType tracker_type)
{
	bool success = true;
	switch(tracker_type)
	{
		case ByDetection: 
		{
			for (int i = 0; i < numCameras; i++)
			{
				// i is the index of camera
				detectWindow = ReceivedImages[i].clone();
				// 如果把success放在前面，则success为false时，函数不会执行
				success =  getContoursAndMoment(i) && success;
				success =  RectifyMarkerPos(i) && success;
				for (int j = 0; j < numMarkers; j++)
				{
					std::cout <<"Inital position: "<< i << j << currentPos[i][j] << std::endl;
				}
			}
			if (success)
			{
				std::cout << "Using Contours to InitTracker succeed" << std::endl;

				TrackerIntialized = true;
			}
		}
		break;
		default:break;
	}
	
	return success;
}

bool Tracker::UpdateTracker(TrackerType tracker_type)
{
	bool success = true;                                                                           
	// using contours to update tracker 
	for(int i=0; i<numCameras; i++)
	{
		
		// j is the index of marker
		for(int j=0; j<numMarkers; j++)
		{
			// use previous position of marker as the center of detectWindowPosition
			// and move detectWindowPosition to left_upper corner
			detectWindowPosition = previousPos[i][j] - cv::Point(int(detectWindowDimX/2), int(detectWindowDimY/2));
			cv::Rect detectRect(detectWindowPosition.x, detectWindowPosition.y, detectWindowDimX, detectWindowDimY);
			detectWindow = ReceivedImages[i](detectRect);
			success = success && getContoursAndMoment(i, j);
		}		
		success = success && RectifyMarkerPos(i);
		for (int j = 0; j < numMarkers; j++)
		{
			std::cout <<i<<j<< currentPos[i][j] << std::endl;
			cv::circle(ReceivedImages[i], currentPos[i][j], 5, cv::Scalar(255, 0, 0));
			cv::rectangle(ReceivedImages[i], cv::Rect(currentPos[i][j].x - detectWindowDimX/2,
				currentPos[i][j].y- detectWindowDimY / 2, detectWindowDimX, detectWindowDimY), cv::Scalar(255, 0, 0));
		}
	}
	return success;
}

// use bubble_sort to rectify Marker Position, from small to big
// from top of image to bottom of image
bool Tracker::RectifyMarkerPos(int camera_index)
{
	int i, j, change=1;
    int len = numMarkers; // length of list to be bubble_sorted
	for (i = 0; i < len - 1 && change!=0; i++)
    {
        change=0;
		for (j = 0; j < len - 1 - i; j++)
		{
			if (currentPos[camera_index][j].y > currentPos[camera_index][j + 1].y)
			{
			std::swap(currentPos[camera_index][j], currentPos[camera_index][j + 1]);
			change = 1;
			}
		}
    }
	return true;
}

bool Tracker::FilterInitalImage()
{
	//
	return true;
}