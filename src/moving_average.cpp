#include "moving_average.hpp"

float* EMA::filter(std::vector<float> angles)
{
	// eura angles指欧拉角，即当前时刻的角度减去上次的角度
	float eura_angles[6];
	for (int i = 0; i < angles.size(); i++)
	{
		eura_angles[i] = weight * angles[i] - weight * S_pre[i];
		S_pre[i] = weight * angles[i] + (1 - weight) * S_pre[i];
	}
	return eura_angles;
}

void EMA::feed(std::vector<float> angles) // feed numbers to establish EMA
{
	if (num_terms == 0)
	{
		for (int i = 0; i < angles.size(); i++)
		{
			S_pre.push_back(angles[i]);
		}
	}
	else
	{
		for (int i = 0; i < angles.size(); i++)
		{
			S_pre[i] = weight * angles[i] + (1 - weight) * S_pre[i];
		}
	}
	num_terms++;
	if (num_terms >= INIT_NUM)
	{
		EMA_established = true; // after INIT_NUM numbers are input, EMA are established
		std::cout << "EMA now established \n\n\a\a" << std::endl;
	}
}