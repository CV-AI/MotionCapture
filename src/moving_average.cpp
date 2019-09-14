#include "moving_average.hpp"

std::vector<double> EMA::filter(std::vector<double> angles)
{
	// eura angles指欧拉角，即当前时刻的角度减去上次的角度
	
	if (num_terms < 2)
	{
		S_pre = angles;
		num_terms++;
	}
	else
	{
		for (int i = 0; i < angles.size(); i++)
		{
			S_pre[i] = weight * angles[i] + (1 - weight) * S_pre[i];
		}
	}
	return S_pre;
}

// 如果已经初始化了就直接返回滤波值
// 否则就返回初始值
std::vector<cv::Vec3f> EMA::filter(std::vector<cv::Vec3f> pnts)
{
	if (num_terms == 0)
	{
		
		Points_pre = pnts;
		num_terms++;
	}
	else
	{
		for (int i = 0; i < pnts.size(); i++)
		{
			Points_pre[i] = weight * pnts[i] + (1 - weight) * Points_pre[i];
		}
	}
	return Points_pre;
}