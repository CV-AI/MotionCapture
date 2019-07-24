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
int Tracker::CorlorsChosen[4];
bool Tracker::getColors = false;
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
		std::vector<cv::Mat> matChannels(3);
		cv::split(image, matChannels);
		CorlorsChosen[0] = static_cast<int>(cv::mean(matChannels[0]).val[0]);
		CorlorsChosen[1] = static_cast<int>(cv::mean(matChannels[1]).val[0]);
		CorlorsChosen[2] = static_cast<int>(cv::mean(matChannels[2]).val[0]);
		getColors = true;
		std::cout << "blue green red: " << CorlorsChosen[0] << " " << CorlorsChosen[1] << " " << CorlorsChosen[2] << std::endl;
	}
}



void Tracker::ColorTheresholding()
{
	for (int j = 0; j < detectWindow.rows; j++)
	{
		uchar*data = detectWindow.ptr<uchar>(j);
		for (int i = 0; i< detectWindow.cols; i++)
		{
			
			if ((abs(data[4 * i] - CorlorsChosen[0]) + abs(data[4 * i + 1] - CorlorsChosen[1]) + abs(data[4 * i + 2] - CorlorsChosen[2]))< threshold)
			{
				data[4 * i] = data[4 * i + 1] = data[4 * i + 2] = 255;
			}
			else data[4 * i] = data[4 * i + 1] = data[4 * i + 2] = 0;
		}
	}
}

// This function using contours to get marker position
bool Tracker:: getContoursAndMoment(int camera_index)
{	
	ColorTheresholding();
	cv::cvtColor(detectWindow, detectWindow, CV_BGRA2GRAY);
	cv::Mat mask(3, 3, CV_8U, cv::Scalar(1));
	cv::morphologyEx(detectWindow, detectWindow, cv::MORPH_CLOSE, mask);

	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(detectWindow, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	int cmin = 20;
	int cmax = 120;
	std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
	// remove contours that are too small or large
	while (itc != contours.end())
	{
		std::cout << "size: " << itc->size() << std::endl;
		if (itc->size() < cmin || itc->size() > cmax)
		{
			itc = contours.erase(itc);
		}
		else itc++;
	}
	std::cout << "The number of contours is " << contours.size() << std::endl;
	if (contours.size()==6 || contours.size()==1)
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
		return true;
	}
	else
	{
		// TODO: 如果使用颜色跟踪失败则需要使用其他跟踪方法
		std::cout << "Contours numbers are wrong"<<std::endl;
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
		for(int i =0; i<4; i++)
		{
			// i is the index of camera
			detectWindow = ReceivedImages[i].clone();
			success = success && getContoursAndMoment(i);
			success = success && RectifyMarkerPos(i);
		}	
		if(success)
		{
			std::cout << "Using Contours to InitTracker succeed"<<std::endl;
			TrackerIntialized = true;
		}
		break;
		default:
		break;
	}
	
	return success;
}

bool Tracker::UpdateTracker(TrackerType tracker_type)
{
	bool success = true;
	// using contours to update tracker 
	for(int i=0; i<4; i++)
	{
		// i is the index of camera
		detectWindow = ReceivedImages[i].clone();
		// j is the index of marker
		for(int j=0; j<6; j++)
		{
			// use previous position of marker as the center of detectWindowPosition
			// and move detectWindowPosition to left_upper corner
			detectWindowPosition = previousPos[i][j] - cv::Point(int(detectWindowDimX/2), int(detectWindowDimY/2));
			cv::Rect detectRect(detectWindowPosition.x, detectWindowPosition.y, detectWindowDimX, detectWindowDimY);
			detectWindow = detectWindow(detectRect);
			success = success && getContoursAndMoment(i);
			success = success && RectifyMarkerPos(i);
		}		
	}
	return success;
}

// use bubble_sort to rectify Marker Position, from small to big
// from top of image to bottom of image
bool Tracker::RectifyMarkerPos(int camera_index)
{
	int i, j, change=1;
    int len = 6; // length of list to be bubble_sorted
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

