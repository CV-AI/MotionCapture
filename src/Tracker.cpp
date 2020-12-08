// 此文件是追踪器的定义文件
// 其负责双侧标记点的追踪 
#include "Tracker.hpp"
#include <algorithm>

Tracker::Tracker():threshold(100),TrackerAutoIntialized(false)
{
	cv::Point momentum[NUM_CAMERAS] = { cv::Point(0,0), cv::Point(0,0),cv::Point(0,0), cv::Point(0,0) };
}

Tracker::~Tracker()
{
}
// 对静态变量进行初始化
cv::Mat Tracker:: image;
cv::Scalar Tracker::CorlorsChosen(0,0,0);
bool Tracker::getColors = false;
cv::Rect Tracker::calibration_region;
cv::Mat Tracker::ReceivedImages[NUM_CAMERAS];
cv::Point2f Tracker::currentPos[NUM_CAMERAS][NUM_MARKERS];
cv::Point2f Tracker::previousPos[NUM_CAMERAS][NUM_MARKERS];
cv::Point2f Tracker::currentPosSet[NUM_CAMERAS][NUM_MARKER_SET];
cv::Point2f Tracker::previousPosSet[NUM_CAMERAS][NUM_MARKER_SET];

// 比较两个coutour的面积（用于contour的排序算法）
bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	double i = fabs(cv::contourArea(cv::Mat(contour1)));
	double j = fabs(cv::contourArea(cv::Mat(contour2)));
	return (i > j); // 从大到小排序
}

bool compareContourHeight(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	float i = cv::minAreaRect(contour1).center.y;
	float j = cv::minAreaRect(contour2).center.y;
	return (i < j); // 从小到大排序，也可以说是从高到低排序
}
// 通过HSV颜色空间进行阈值化处理，能较好地避免光照影响
void Tracker::ColorThresholding(int marker_index)
{
	cv::Mat rangeRes = cv::Mat::zeros(detectWindow.size(), CV_8UC1);
	//cv::cvtColor(detectWindow, detectWindow, CV_RGB2HSV);
	cv::inRange(detectWindow, LOWER_RED, UPPER_RED, rangeRes);
	detectWindow = rangeRes;
}

// 通过HSV颜色空间进行阈值化处理，能较好地避免光照影响
void Tracker::ColorThresholding()
{
	cv::Mat rangeRes = cv::Mat::zeros(detectWindow_Initial.size(), CV_8UC1);
	//cv::cvtColor(detectWindow_Initial, detectWindow_Initial, CV_RGB2HSV);
	cv::inRange(detectWindow_Initial, LOWER_RED, UPPER_RED, rangeRes);
	detectWindow_Initial = rangeRes;
}

// 通过contour获取六个标记点的起始位置
bool Tracker:: initMarkerPosition(int camera_index)
{	
	ColorThresholding();
	//cv::erode(detectWindow_Initial, detectWindow_Initial, mask_erode);
	/*cv::namedWindow("detectwindow_erode", 0);
	cv::imshow("detectwindow_erode", detectWindow_Initial);*/
	cv::morphologyEx(detectWindow_Initial, detectWindow_Initial, cv::MORPH_CLOSE, mask);
	cv::namedWindow("detectwindow", 0);
	cv::imshow("detectwindow", detectWindow_Initial);
	calibration_region = cv::selectROI("detectwindow", detectWindow_Initial);
	cv::Mat region_mask = cv::Mat::zeros(detectWindow_Initial.size(), CV_8UC1);
	region_mask(calibration_region).setTo(255);
	cv::Mat masked_window;
	detectWindow_Initial.copyTo(masked_window, region_mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(masked_window, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
	
	if (contours.size() == NUM_MARKERS)
	{
		// 找到contour的boundingRect的中心
		for (int i = 0; i < 6; i++)
		{
			if (_PRINT_PROCESS)
			{
				std::cout << "contour size" << contours[i].size() << std::endl;
			}
			currentPos[camera_index][i] = cv::minAreaRect(contours[i]).center;
		}
		RectifyMarkerPos(camera_index);
		for (int marker_set = 0; marker_set < NUM_MARKER_SET; marker_set++)
		{
			currentPosSet[camera_index][marker_set] = 0.5 * (currentPos[camera_index][2 * marker_set] + currentPos[camera_index][2 * marker_set + 1]);
		}
		// cv::drawContours(ReceivedImages[camera_index], contours, -1, cv::Scalar(0, 255, 0), 5);//-1 means draw all contours
		cv::imshow("detectwindow", masked_window);
		cv::waitKey(0);
		cv::destroyWindow("detectwindow");
		return true;
	}
	else
	{
		std::cout << "Camera " <<camera_index<<" Incorrect Number of Contours: " << contours.size() << std::endl;
		return false;
	}
}

//更新标记点、标记点对的位置
bool Tracker::updateMarkerPosition(int camera_index, int marker_set)
{
	ColorThresholding(camera_index);
	//cv::erode(detectWindow, detectWindow, mask_erode);
	cv::morphologyEx(detectWindow, detectWindow, cv::MORPH_CLOSE, mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(detectWindow, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	if (contours.size() ==2)
	{
		// 如果只有两个标记的在检测框中
		// 那么按照高度排序即可
		std::sort(contours.begin(), contours.end(), compareContourHeight);
		cv::Point2f center_0 = cv::minAreaRect(contours[0]).center;
		cv::Point2f center_1 = cv::minAreaRect(contours[1]).center;
		
		currentPos[camera_index][2 * marker_set] = center_0 + cv::Point2f(detectPosition);
		currentPos[camera_index][2 * marker_set + 1] = center_1 + cv::Point2f(detectPosition);
	}
	else if (contours.size() == 3)
	{
		// 如果出现了三个标记点，很可能是因为足部的标记点进入了小腿部分的检测框
		// 这时，可以确定最高的那个点是属于小腿的
		std::sort(contours.begin(), contours.end(), compareContourHeight);
		cv::Point2f center_0 = cv::minAreaRect(contours[0]).center;
		currentPos[camera_index][2 * marker_set] = center_0 + cv::Point2f(detectPosition);

		cv::Point2f p0 = cv::minAreaRect(contours[1]).center + cv::Point2f(detectPosition);
		cv::Point2f p1 = cv::minAreaRect(contours[2]).center + cv::Point2f(detectPosition);
		cv::Point2f prev = previousPos[camera_index][2 * marker_set + 1];
		// 然后根据前一帧的位置确定第二点（更靠近前一帧的是正确的）
		if (pointDist(p0, prev) < pointDist(p1, prev))
		{
			currentPos[camera_index][2 * marker_set + 1] = p0;
		}
		else
		{
			currentPos[camera_index][2 * marker_set + 1] = p1;
		}
	}
	else
	{
		// TODO: 如果使用颜色跟踪失败则需要使用其他跟踪方法
		std::cout << "Number of contours is wrong:  " << contours.size() << std::endl;
		return false;
	}
	currentPosSet[camera_index][marker_set] = 0.5 * (currentPos[camera_index][2 * marker_set] + currentPos[camera_index][2 * marker_set + 1]);
	return true;
}

// 为了方便使用，初始化tracker时都应该实际使用ByDetection（使用其他的tracker都需要自行画出框图）
bool Tracker::InitTracker(TrackerType tracker_type)
{
	bool success = true;
	switch(tracker_type)
	{
		case ByDetection: 
		{
			for (int i = 0; i < NUM_CAMERAS; i++)
			{
				// i is the index of camera
				detectWindow_Initial = ReceivedImages[i].clone();
				// 如果把success放在前面，则success为false时，函数不会执行
				success =  initMarkerPosition(i) && success;
				for (int j = 0; j < NUM_MARKERS; j++)
				{
					std::cout <<"Inital position: "<< i << j << currentPos[i][j] << std::endl;
				}
			}
			if (success)
			{
				std::cout << "Using Contours to InitTracker succeed" << std::endl;

				TrackerAutoIntialized = true;
			}
		}
		break;
		default:break;
	}
	// 检查数据正确性
	for (int i = 0; i < NUM_CAMERAS; i++)
	{
		for (int j = 0; j < NUM_MARKERS; j++)
		{
			cv::Point point = currentPos[i][j];
			if (point.x <= 0 || point.y <= 0 || point.x>=2048 || point.y>=2048)
			{
				TrackerAutoIntialized = false;
				success = false;
			}
		}
	}
	
	return success;
}

// 校正标记点的位置
bool Tracker::RectifyMarkerPos(int camera_index)
{
	int i, j, change=1;
	// 对于0-3号标记点，利用y值判断label 是否准确
	for (i = 0; i < NUM_MARKERS - 1 && change!=0; i++)
    {
        change=0;
		for (j = 0; j < NUM_MARKERS - 1 - i; j++)
		{
			if (currentPos[camera_index][j].y > currentPos[camera_index][j + 1].y)
			{
			std::swap(currentPos[camera_index][j], currentPos[camera_index][j + 1]);
			change = 1;
			}
		}
    }
	// 对于0，1号相机来说，判断向量4->5标记点是否正确
	if (camera_index < 2)
	{
		if ((currentPos[camera_index][4].x < currentPos[camera_index][5].x) 
					/*&& (currentPos[camera_index][4].y > currentPos[camera_index][5].y)*/)
		{
			std::swap(currentPos[camera_index][4], currentPos[camera_index][5]);
		}
	}
	// 对于2，3号相机来说
	else
	{
		if (currentPos[camera_index][4].x > currentPos[camera_index][5].x /*&& currentPos[camera_index][4].y > currentPos[camera_index][5].y*/)
		{
			std::swap(currentPos[camera_index][4], currentPos[camera_index][5]);
		}
	}
	return true;
}

bool Tracker::FilterInitialImage()
{
	//TODO: clearify images for tracker initialization
	return true;
}

float pointDist(const cv::Point2f& p0, const cv::Point2f& p1)
{
	// we don't calculate sqaure root because it's not important to our goal
	return pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2);
}

// 多线程同时更新追踪器
DWORD WINAPI UpdateTracker(LPVOID lpParam)
{
	TrackerParameters para = *((TrackerParameters*)lpParam);
	int camera_index = para.camera_index;
	Tracker* trackerPtr = para.trackerPtr;
	TrackerType tracker_type = para.tracker_type;
	bool success = true;
	cv::Point detectRegionPosition;
	cv::Rect detectRect;
	switch (tracker_type)
	{
	case ByDetection:
		for (int j = 0; j < NUM_MARKER_SET; j++)
		{
			int window_size_x = (*trackerPtr).detectWindowDim[camera_index][j].x;
			int window_size_y = (*trackerPtr).detectWindowDim[camera_index][j].y;
			// 将检测窗口中心的位置设为上一帧标记对位置（再加上一个动量，以模拟标记点的运动）
			(*trackerPtr).detectPosition = (*trackerPtr).previousPosSet[camera_index][j] + (*trackerPtr).momentum[j] - cv::Point2f(int(window_size_x*0.5), int(window_size_y * 0.5));
			// 保证ROI不超出图像范围
			if ((*trackerPtr).detectPosition.x < 0)
			{
				(*trackerPtr).detectPosition.x = 0;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			if ((*trackerPtr).detectPosition.y < 0)
			{
				(*trackerPtr).detectPosition.y = 0;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			if ((*trackerPtr).detectPosition.x+ window_size_x > (*trackerPtr).ReceivedImages[camera_index].cols)
			{
				window_size_x = (*trackerPtr).ReceivedImages[camera_index].cols - (*trackerPtr).detectPosition.x;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			if ((*trackerPtr).detectPosition.y + window_size_y > (*trackerPtr).ReceivedImages[camera_index].rows)
			{
				window_size_y = (*trackerPtr).ReceivedImages[camera_index].rows - (*trackerPtr).detectPosition.y;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			
			detectRect = cv::Rect((*trackerPtr).detectPosition.x, (*trackerPtr).detectPosition.y, window_size_x, window_size_y);
			
			(*trackerPtr).detectWindow = (*trackerPtr).ReceivedImages[camera_index](detectRect).clone(); 
			success = (*trackerPtr).updateMarkerPosition(camera_index,j) && success;
			(*trackerPtr).momentum[j] = weight*((*trackerPtr).currentPosSet[camera_index][j] - (*trackerPtr).previousPosSet[camera_index][j]);
		}
		break;
	case CV_KCF:
		break;
	case ByColor:
		break;
	default:
		break;
	}
	return success;
}
