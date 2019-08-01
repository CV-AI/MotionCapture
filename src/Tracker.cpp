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
cv::Rect Tracker::calibration_region;
cv::Mat Tracker::ReceivedImages[NUM_CAMERAS];
cv::Point Tracker::currentPos[NUM_CAMERAS][NUM_MARKERS];
cv::Point Tracker::previousPos[NUM_CAMERAS][NUM_MARKERS];
// this function compare the areas of two contours
bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
	double i = fabs(cv::contourArea(cv::Mat(contour1)));
	double j = fabs(cv::contourArea(cv::Mat(contour2)));
	return (i > j); // 从大到小排序
}

// initialize with whole image, so detectPosition_Initial is (0, 0)
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

// this function get the region where lies the markers(to exclude noise)
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

void Tracker::ColorThresholding(int marker_index)
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

void Tracker::ColorThresholding()
{
	cv::cvtColor(detectWindow_Initial, detectWindow_Initial, CV_RGB2HSV);
	cv::Mat rangeRes = cv::Mat::zeros(detectWindow_Initial.size(), CV_8UC1);
	cv::inRange(detectWindow_Initial, cv::Scalar(MIN_H_RED, 100, 80), cv::Scalar(MAX_H_RED, 255, 255), rangeRes);
	detectWindow_Initial = rangeRes;
}

// This function using contours to get all of six marker positions for one camera
bool Tracker:: getContoursAndMoment(int camera_index)
{	
	ColorThresholding();
	
	//cv::fastNlMeansDenoising(detectWindow_Initial, detectWindow_Initial);
	//cv::Mat detectCopy = detectWindow_Initial.clone();
	//cv::cvtColor(detectWindow_Initial, detectWindow_Initial, CV_BGRA2GRAY);
	cv::Mat mask(9, 9, CV_8U, cv::Scalar(1));
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
	cv::drawContours(ReceivedImages[camera_index], contours, -1, cv::Scalar(150, 100, 0), 2);//-1 means draw all contours
	// remove contours that are too small or large
	//while (itc != contours.end())
	//{
	//	std::cout << "Camera "<<camera_index<<" "<<itc->size() << std::endl;
	//	//std::cout << "size: " << itc->size() << std::endl;
	//	if (itc->size() < cmin || itc->size() > cmax)
	//	{
	//		itc = contours.erase(itc);
	//	}
	//	else itc++;
	//}
	
	std::sort(contours.begin(), contours.end(), compareContourAreas);
	//contours = std::vector<std::vector<cv::Point>>(contours.begin(), contours.begin() + 6);
	
	if (contours.size() >= 6)
	{
		//std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();
	// 找到contour的boundingRect的中心
		for (int i = 0; i < 6; i++)
		{
			std::cout << "contour size" << contours[i].size() << std::endl;
			cv::Rect br = cv::boundingRect(contours[i]);
			int cx = br.x + br.width / 2; int cy = br.y + br.height / 2;
			currentPos[camera_index][i] = cv::Point(cx + detectPosition_Initial.x, cy + detectPosition_Initial.y);
		}
		//int i = 0;
		//while (it != contours.end())
		//{

		//	cv::Moments mom = cv::moments(cv::Mat(*it++));
		//	// 因为检测窗口不是全部画面，所以要加上检测窗口在全部画面的位置
		//	currentPos[camera_index][i] = cv::Point(mom.m10 / mom.m00+detectPosition_Initial.x, mom.m01 / mom.m00+ detectPosition_Initial.y);
		//	i++;
		//}
		cv::drawContours(ReceivedImages[camera_index], contours, -1, cv::Scalar(0, 255, 0), 5);//-1 means draw all contours
		cv::namedWindow("detectwindow", 0);
		cv::imshow("detectwindow", masked_window);
		cv::waitKey(0);
		return true;

	}
	else
	{
		std::cout << "No enough contours: " << contours.size() << std::endl;
	}
}

// This function get the marker point for specific marker in a specific camera
bool Tracker::getContoursAndMoment(int camera_index, int marker_index)
{
	ColorThresholding(camera_index);
	//cv::Mat detectCopy = detectWindow_Initial.clone();
	//cv::cvtColor(detectWindow_Initial, detectWindow_Initial, CV_BGRA2GRAY);
	cv::Mat mask(5, 5, CV_8U, cv::Scalar(1));
	cv::morphologyEx(detectWindow, detectWindow, cv::MORPH_CLOSE, mask);
	std::vector<std::vector<cv::Point>>contours;
	cv::findContours(detectWindow, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
	// remove contours that are too small or large
	//while (itc != contours.end())
	//{
	//	std::cout << itc->size() << std::endl;
	//	//std::cout << "size: " << itc->size() << std::endl;
	//	if (itc->size() < cmin || itc->size() > cmax)
	//	{
	//		itc = contours.erase(itc);
	//	}
	//	else itc++;
	//}
	/*cv::namedWindow("detectWindow", 0);
	cv::imshow("detectWindow", detectCopy);
	cv::waitKey(1);*/
	std::sort(contours.begin(), contours.end(), compareContourAreas);
	//contours = std::vector<std::vector<cv::Point>>(contours.begin(), contours.begin() + 1);
	if (contours.size() >0)
	{
		cv::Rect br = cv::boundingRect(contours[0]);
		int cx = br.x + br.width / 2; int cy = br.y + br.height / 2;
		currentPos[camera_index][marker_index] = cv::Point(cx + detectPosition_Initial.x, cy + detectPosition_Initial.y);
		
		//std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();

		//cv::Moments mom = cv::moments(cv::Mat(*it++));
		//// 因为检测窗口不是全部画面，所以要加上检测窗口在全部画面的位置
		//currentPos[camera_index][marker_index] = cv::Point(mom.m10 / mom.m00 + detectPosition_Initial.x, mom.m01 / mom.m00 + detectPosition_Initial.y);

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
				success =  getContoursAndMoment(i) && success;
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
	// see if data were correct
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

// use bubble_sort to rectify Marker Position, from small to big
// from top of image to bottom of image
bool Tracker::RectifyMarkerPos(int camera_index)
{
	int i, j, change=1;
    int len = NUM_MARKERS; // length of list to be bubble_sorted
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

bool Tracker::FilterInitialImage()
{
	//TODO: clearify images for tracker initialization
	return true;
}

#if defined (_WIN32)
DWORD WINAPI UpdateTracker(LPVOID lpParam)
{
#endif
	TrackerParameters para = *((TrackerParameters*)lpParam);
	int marker_index = para.marker_index;
	Tracker* trackerPtr = para.trackerPtr;
	TrackerType tracker_type = para.tracker_type;
	bool success = true;
	cv::Point detectRegionPosition;

	switch (tracker_type)
	{
	case ByDetection:
		// using contours to update tracker 
		for (int i = 0; i < NUM_CAMERAS; i++)
		{
			// use previous position of marker as the center of detectPosition_Initial
			// and move detectPosition_Initial to left_upper corner
			(*trackerPtr).detectPosition = (*trackerPtr).previousPos[i][marker_index] - cv::Point(int((*trackerPtr).detectWindowDimX / 2), int((*trackerPtr).detectWindowDimY / 2));
			cv::Rect detectRect((*trackerPtr).detectPosition.x, (*trackerPtr).detectPosition.y, (*trackerPtr).detectWindowDimX, (*trackerPtr).detectWindowDimY);
			(*trackerPtr).detectWindow = (*trackerPtr).ReceivedImages[i](detectRect).clone(); // 
			success = (*trackerPtr).getContoursAndMoment(i, marker_index) && success;
			std::cout << i << marker_index << (*trackerPtr).currentPos[i][marker_index] << std::endl;
			cv::circle((*trackerPtr).ReceivedImages[i], (*trackerPtr).currentPos[i][marker_index], 5, cv::Scalar(255, 0, 0));
			/*cv::rectangle(ReceivedImages[i], cv::Rect(currentPos[i][marker_index].x - detectWindowDimX / 2,
				currentPos[i][marker_index].y - detectWindowDimY / 2, detectWindowDimX, detectWindowDimY), cv::Scalar(255, 0, 0));*/
			success = (*trackerPtr).RectifyMarkerPos(i) && success;
		}
		break;
	case CV_KCF:
		break;
	case ByColor:
		// using contours to update tracker 
		for (int i = 0; i < NUM_CAMERAS; i++)
		{

			// use previous position of marker as the center of detectPosition_Initial
			// and move detectPosition_Initial to left_upper corner
			(*trackerPtr).detectPosition = (*trackerPtr).previousPos[i][marker_index] - cv::Point(int((*trackerPtr).detectWindowDimX / 2), int((*trackerPtr).detectWindowDimY / 2));
			cv::Rect detectRect((*trackerPtr).detectPosition.x, (*trackerPtr).detectPosition.y, (*trackerPtr).detectWindowDimX, (*trackerPtr).detectWindowDimY);
			(*trackerPtr).detectWindow = (*trackerPtr).ReceivedImages[i](detectRect).clone(); // 
			//tracker.ColorTheresholding();
			//for(int j)

			std::cout << i << marker_index << (*trackerPtr).currentPos[i][marker_index] << std::endl;
			cv::circle((*trackerPtr).ReceivedImages[i], (*trackerPtr).currentPos[i][marker_index], 5, cv::Scalar(255, 0, 0));
			cv::rectangle((*trackerPtr).ReceivedImages[i], cv::Rect((*trackerPtr).currentPos[i][marker_index].x - (*trackerPtr).detectWindowDimX / 2,
				(*trackerPtr).currentPos[i][marker_index].y - (*trackerPtr).detectWindowDimY / 2,
				(*trackerPtr).detectWindowDimX, (*trackerPtr).detectWindowDimY), cv::Scalar(255, 0, 0));
			success = (*trackerPtr).RectifyMarkerPos(i) && success;
		}
		break;
	default:
		break;
	}

	return success;
}
