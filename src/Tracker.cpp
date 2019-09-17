// 此文件用于初始化跟踪 
#include "Tracker.hpp"
#include <algorithm>


Tracker::Tracker():detectWindowDimX(120), detectWindowDimY(120),threshold(100),TrackerAutoIntialized(false)
{
	cv::Point momentum[NUM_CAMERAS] = { cv::Point(0,0), cv::Point(0,0),cv::Point(0,0), cv::Point(0,0) };
}


Tracker::~Tracker()
{
}
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

// 获取鼠标划过区域的平均颜色
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
		assert(image.channels() == 3); // image must be three channels
		CorlorsChosen = cv::mean(image);
		getColors = true;
		std::cout << "blue green red: " << CorlorsChosen << std::endl;
	}
}

// 获取六个标记点所在区域（以排除其他噪声点的干扰）
void Tracker::Mouse_getRegion(int event, int x, int y, int, void*)
{
	static cv::Point origin;
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = cv::Point(x, y);
		break;
	case CV_EVENT_LBUTTONUP:
		calibration_region.x = MIN(x, origin.x);
		calibration_region.y = MIN(y, origin.y);
		calibration_region.width = abs(x - origin.x);
		calibration_region.height = abs(y - origin.y);
	}
}

// 通过HSV颜色空间进行阈值化处理，能较好地避免光照影响
void Tracker::ColorThresholding(int marker_index)
{
	cv::cvtColor(detectWindow, detectWindow, CV_RGB2HSV);
	cv::Mat rangeRes = cv::Mat::zeros(detectWindow.size(), CV_8UC1);
	cv::inRange(detectWindow, cv::Scalar(MIN_H_RED, MIN_SATURATION, MIN_VALUE), cv::Scalar(MAX_H_RED, MAX_SATURATION, MAX_VALUE), rangeRes);
	detectWindow = rangeRes;
}

// 通过HSV颜色空间进行阈值化处理，能较好地避免光照影响
void Tracker::ColorThresholding()
{
	cv::cvtColor(detectWindow_Initial, detectWindow_Initial, CV_RGB2HSV);
	cv::Mat rangeRes = cv::Mat::zeros(detectWindow_Initial.size(), CV_8UC1);
	cv::inRange(detectWindow_Initial, cv::Scalar(MIN_H_RED, MIN_SATURATION, MIN_VALUE), cv::Scalar(MAX_H_RED, MAX_SATURATION, MAX_VALUE), rangeRes);
	detectWindow_Initial = rangeRes;
}

// 通过contour获取六个标记点的起始位置
bool Tracker:: initMarkerPosition(int camera_index)
{	
	ColorThresholding();
	
	cv::Mat mask(5, 5, CV_8U, cv::Scalar(1));
	cv::morphologyEx(detectWindow_Initial, detectWindow_Initial, cv::MORPH_CLOSE, mask);
	cv::namedWindow("detectwindow", 0);
	cv::setMouseCallback("detectwindow", Mouse_getRegion, 0);
	cv::imshow("detectwindow", detectWindow_Initial);
	cv::waitKey(0);
	cv::Mat region_mask = cv::Mat::zeros(detectWindow_Initial.size(), CV_8UC1);
	region_mask(calibration_region).setTo(255);
	cv::Mat masked_window;
	detectWindow_Initial.copyTo(masked_window, region_mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(masked_window, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
	//cv::drawContours(ReceivedImages[camera_index], contours, -1, cv::Scalar(150, 100, 0), 2);//-1 means draw all contours
	
	std::sort(contours.begin(), contours.end(), compareContourAreas);
	//contours = std::vector<std::vector<cv::Point>>(contours.begin(), contours.begin() + 6);
	
	if (contours.size() >= 6)
	{
		//std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();
		// 找到contour的boundingRect的中心
		for (int i = 0; i < 6; i++)
		{
			std::cout << "contour size" << contours[i].size() << std::endl;
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
		std::cout << "No enough contours: " << contours.size() << std::endl;
		return false;
	}
}

//获取标记点、标记点对的位置
bool Tracker::updateMarkerPosition(int camera_index, int marker_set)
{
	ColorThresholding(camera_index);
	cv::Mat mask(5, 5, CV_8U, cv::Scalar(1));
	cv::morphologyEx(detectWindow, detectWindow, cv::MORPH_CLOSE, mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(detectWindow, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	/*cv::namedWindow("detectWindow", 0);
	cv::imshow("detectWindow", detectCopy);
	cv::waitKey(1);*/
	std::sort(contours.begin(), contours.end(), compareContourAreas);
	if (contours.size() >=2)
	{
		/*cv::Rect br_u = cv::boundingRect(contours[0]);
		int center_x_u = cvRound( br_u.x + br_u.width * 0.5); 
		int center_y_u = cvRound(br_u.y + br_u.height * 0.5);
		cv::Rect br_l = cv::boundingRect(contours[1]);
		int center_x_l = cvRound(br_l.x + br_l.width * 0.5);
		int center_y_l = cvRound(br_l.y + br_l.height * 0.5);*/
		cv::Point2f center_0 = cv::minAreaRect(contours[0]).center;
		cv::Point2f center_1 = cv::minAreaRect(contours[1]).center;
		
		if (center_0.y < center_1.y)
		{
			currentPos[camera_index][2 * marker_set] = center_0 + cv::Point2f(detectPosition);
			currentPos[camera_index][2 * marker_set + 1] = center_1 + cv::Point2f(detectPosition);

		}
		else
		{
			currentPos[camera_index][2 * marker_set + 1] = center_0 + cv::Point2f(detectPosition);
			currentPos[camera_index][2 * marker_set] = center_1 + cv::Point2f(detectPosition);
		}
		currentPosSet[camera_index][marker_set] = 0.5 * (currentPos[camera_index][2 * marker_set] + currentPos[camera_index][2 * marker_set + 1]);
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
			for (int i = 0; i < NUM_CAMERAS; i++)
			{
				// i is the index of camera
				detectWindow_Initial = ReceivedImages[i].clone();
				// 如果把success放在前面，则success为false时，函数不会执行
				success =  initMarkerPosition(i) && success;
				success =  RectifyMarkerPos(i) && success;
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

DWORD WINAPI UpdateTracker(LPVOID lpParam)
{
	TrackerParameters para = *((TrackerParameters*)lpParam);
	int camera_index = para.camera_index;
	Tracker* trackerPtr = para.trackerPtr;
	TrackerType tracker_type = para.tracker_type;
	bool success = true;
	cv::Point detectRegionPosition;
	cv::Rect detectRect;
	int window_size_x = (*trackerPtr).detectWindowDimX;
	int window_size_y = (*trackerPtr).detectWindowDimY;
	switch (tracker_type)
	{
	case ByDetection:
		for (int j = 0; j < NUM_MARKER_SET; j++)
		{
			// 将检测窗口中心的位置设为上一帧标记对位置（再加上一个动量，以模拟标记点的运动）
			(*trackerPtr).detectPosition = cv::Point2i((*trackerPtr).previousPosSet[camera_index][j]) + (*trackerPtr).momentum[j] - cv::Point(int((*trackerPtr).detectWindowDimX*0.5), int((*trackerPtr).detectWindowDimY * 0.5));
			// 保证ROI不超出图像范围
			if ((*trackerPtr).detectPosition.x < 0)
			{
				(*trackerPtr).detectPosition.x = 0;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << "\n";
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			if ((*trackerPtr).detectPosition.y < 0)
			{
				(*trackerPtr).detectPosition.y = 0;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << "\n";
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			if ((*trackerPtr).detectPosition.x+ (*trackerPtr).detectWindowDimX > (*trackerPtr).ReceivedImages[camera_index].cols)
			{
				window_size_x = (*trackerPtr).ReceivedImages[camera_index].cols - (*trackerPtr).detectPosition.x;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << "\n";
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << std::endl;
			}
			if ((*trackerPtr).detectPosition.y + (*trackerPtr).detectWindowDimY > (*trackerPtr).ReceivedImages[camera_index].rows)
			{
				window_size_y = (*trackerPtr).ReceivedImages[camera_index].rows - (*trackerPtr).detectPosition.y;
				std::cerr << "-----Tracker roi exceeds image limit, rectified. Please check camera orientation!!!!\a\a" << "\n";
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
