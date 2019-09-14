#ifndef MOVING_AVERAGE_HEADER
#define MOVING_AVERAGE_HEADER
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
// exponential moving average, see https://zh.wikipedia.org/wiki/%E7%A7%BB%E5%8B%95%E5%B9%B3%E5%9D%87

class EMA
{
public:
	int num_terms = 0;
	double weight = 0.08;
	std::vector<double> S_pre;
	std::vector<cv::Vec3f> Points_pre;
	
	std::vector<double> filter(std::vector<double> angles);
	std::vector<cv::Vec3f> filter(std::vector<cv::Vec3f> pnts);
};

#endif // !MOVING_AVERAGE_HEADER



