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
bool Tracker::getColors = false;
cv::Rect Tracker::calibration_region;
cv::Point2i Tracker::detectWindowDim[NUM_CAMERAS][NUM_MARKER_SET] = {
										{{60, 120}, {65, 100}, {70, 80} },
										{{60, 120}, {65, 100}, {70, 80} },
										{{60, 120}, {60, 110}, {80, 80} },
										{{60, 120}, {60, 110}, {80, 80} } };
cv::Point2i Tracker::detectWindowDimMin[NUM_CAMERAS][NUM_MARKER_SET] = {
										{{60, 80}, {65, 80}, {70, 80} },
										{{60, 80}, {65, 80}, {70, 80} },
										{{60, 80}, {60, 80}, {80, 80} },
										{{60, 80}, {60, 80}, {80, 80} } };
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

bool compareContourX(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	float i = cv::minAreaRect(contour1).center.x;
	float j = cv::minAreaRect(contour2).center.x;
	return (i < j); // 从小到大排序，也可以说是从左到右
}

bool Tracker::autoInitMarkerPosition(int camera_index)
{
	cv::Mat binary = colorThresholding(detectWindow_Initial, false);
	cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, mask);
	cv::namedWindow("detectwindow", 0);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	// 从左到右将contour排序
	std::sort(contours.begin(), contours.end(), compareContourX);
	// 对于0,1号相机，需要反转这个顺序
	if (camera_index < 2)
	{
		std::reverse(contours.begin(), contours.end());
	}
	// 找到contour的boundingRect的中心
	std::vector<cv::Point2f> selected_contour_centers;
	if (contours.size() < NUM_MARKERS)
	{
		std::wcerr << "There is no enough contours" << std::endl;
		return false;
	}
	for (int i = 0; i < NUM_MARKERS; i++)
	{
		if (_PRINT_PROCESS)
		{
			std::cout << "contour size" << contours[i].size() << std::endl;
		}
		currentPos[camera_index][i] = cv::minAreaRect(contours[i]).center;
		selected_contour_centers.push_back(currentPos[camera_index][i]);
	}
	cv::Rect box = cv::boundingRect(selected_contour_centers);
	cv::rectangle(binary, box, cv::Scalar(255,255,255));
	cv::imshow("detectwindow", binary);
	int key = cv::waitKey(0);
	cv::destroyWindow("detectwindow");
	if (key == 32) // press space
	{
		RectifyMarkerPos(camera_index);
		for (int marker_set = 0; marker_set < NUM_MARKER_SET; marker_set++)
		{
			currentPosSet[camera_index][marker_set] = 0.5 * (currentPos[camera_index][2 * marker_set] + currentPos[camera_index][2 * marker_set + 1]);
		}
		std::cout << "Succeed to auto initialize marker position for camera " << camera_index << std::endl;
		return true;
	}
	else
	{
		std::cout << "Failed to auto initialize marker postion for camera " << camera_index << ".\n";
		std::cout << "Turn to manual..." << std::endl;
		return false;
	}
}
// 通过contour获取六个标记点的起始位置
bool Tracker:: manualInitMarkerPosition(int camera_index)
{	
	cv::Mat binary = colorThresholding(detectWindow_Initial, false);
	cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, mask);
	cv::namedWindow("detectwindow", 0);
	cv::imshow("detectwindow", binary);
	calibration_region = cv::selectROI("detectwindow", binary);
	cv::Mat region_mask = cv::Mat::zeros(detectWindow_Initial.size(), CV_8UC1);
	region_mask(calibration_region).setTo(255);
	cv::Mat masked_window;
	binary.copyTo(masked_window, region_mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(masked_window, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	if (contours.size() == NUM_MARKERS)
	{
		// 找到contour的boundingRect的中心
		for (int i = 0; i < NUM_MARKERS; i++)
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
	cv::Mat binary = colorThresholding(detectWindow, true);
	//cv::erode(detectWindow, detectWindow, mask_erode);
	cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	if (contours.size() ==2)
	{
		// 如果只有两个标记的在检测框中
		// 那么按照高度排序即可
		std::sort(contours.begin(), contours.end(), compareContourHeight);
		cv::Point2f center_0, center_1;
		float _;
		cv::minEnclosingCircle(contours[0], center_0, _);
		cv::minEnclosingCircle(contours[1], center_1, _);
		
		currentPos[camera_index][2 * marker_set] = center_0 + cv::Point2f(detectPosition);
		currentPos[camera_index][2 * marker_set + 1] = center_1 + cv::Point2f(detectPosition);
	}
	else if (contours.size() == 3)
	{
		// 如果出现了三个标记点，很可能是因为足部的标记点进入了小腿部分的检测框
		// 这时，可以确定最高的那个点是属于小腿的
		std::sort(contours.begin(), contours.end(), compareContourHeight);
		cv::Point2f center_0, p1, p2;
		float _;
		cv::minEnclosingCircle(contours[0], center_0, _);
		currentPos[camera_index][2 * marker_set] = center_0 + cv::Point2f(detectPosition);

		cv::minEnclosingCircle(contours[1], p1, _);
		cv::minEnclosingCircle(contours[2], p2, _);
		p1 = p1 + cv::Point2f(detectPosition);
		p2 = p2 + cv::Point2f(detectPosition);
		cv::Point2f prev = previousPos[camera_index][2 * marker_set + 1];
		// 然后根据前一帧的位置确定第二点（更靠近前一帧的是正确的）
		if (pointDist(p1, prev) < pointDist(p2, prev))
		{
			currentPos[camera_index][2 * marker_set + 1] = p1;
		}
		else
		{
			currentPos[camera_index][2 * marker_set + 1] = p2;
		}
	}
	else
	{
		// TODO: 如果使用颜色跟踪失败则需要使用其他跟踪方法
		std::cout << "Number of contours is wrong:  " << contours.size() << std::endl;
		//char* file = new char[12];
		//sprintf(file, "raw_c%im%i.jpg", camera_index, marker_set);
		//cv::imwrite(file, detectWindow);
		//sprintf(file, "bin_c%im%i.jpg", camera_index, marker_set);
		//cv::imwrite(file, binary);
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
				if (autoInitMarkerPosition(i))
				{
					continue;
				}
				else
				{
					// 如果把success放在前面，则success为false时，函数不会执行
					success = manualInitMarkerPosition(i) && success;
				}
			}
			for (int i = 0; i < NUM_CAMERAS; i++)
			{
				for (int j = 0; j < NUM_MARKERS; j++)
				{
					std::cout << "Inital position: " << i << j << currentPos[i][j] << std::endl;
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
		if (currentPos[camera_index][4].x > currentPos[camera_index][5].x)
		{
			
			std::swap(currentPos[camera_index][4], currentPos[camera_index][5]);
		}
	}
	return true;
}

void Tracker::updateDetectWindowDims()
{
	for (int camera = 0; camera < NUM_CAMERAS; camera++)
	{
		for (int marker_set = 0; marker_set < NUM_MARKER_SET; marker_set++)
		{
			int dy = abs(currentPos[camera][marker_set*2].y - currentPos[camera][marker_set*2+1].y);
			int dx = abs(currentPos[camera][marker_set*2].x - currentPos[camera][marker_set*2+1].x);
			detectWindowDim[camera][marker_set].x = dx + DETECT_WINDOW_BOUNDRY;
			detectWindowDim[camera][marker_set].y = dy + DETECT_WINDOW_BOUNDRY;
		}
	}
}

float pointDist(const cv::Point2f& p0, const cv::Point2f& p1)
{
	// 没有计算开方，因为没有必要
	return pow(p0.x-p1.x,2)+pow(p0.y-p1.y,2);
}

cv::Mat colorThresholding(const cv::Mat& color_img, bool tracking)
{
	cv::Mat rangeRes = cv::Mat::zeros(color_img.size(), CV_8UC1);
	if (tracking)
	{
		cv::inRange(color_img, LOWER_RED_TRACK, UPPER_RED, rangeRes);
	}
	else
	{
		cv::inRange(color_img, LOWER_RED, UPPER_RED, rangeRes);
	}
	return rangeRes;
}

// 多线程同时更新追踪器
DWORD WINAPI UpdateTracker(LPVOID lpParam)
{
	TrackerParameters para = *((TrackerParameters*)lpParam);
	int camera_index = para.camera_index;
	Tracker* trackerPtr = para.trackerPtr;
	TrackerType tracker_type = para.tracker_type;
	bool success = true;
	DWORD error_code = 9;
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
			success = (*trackerPtr).updateMarkerPosition(camera_index,j);
			if (!success)
			{
				// 如果error_code是默认状态，那么这是第一次发生错误
				if (error_code == 9)
				{
					error_code = j;
				}
				else
				{
					error_code = error_code * 10 + j;
				}
			}
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
	return error_code;
}
