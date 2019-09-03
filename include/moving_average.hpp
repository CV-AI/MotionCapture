#ifndef MOVING_AVERAGE_HEADER
#define MOVING_AVERAGE_HEADER
#include <vector>
#include <iostream>
// exponential moving average, see https://zh.wikipedia.org/wiki/%E7%A7%BB%E5%8B%95%E5%B9%B3%E5%9D%87

class EMA
{
	double eura_angles[6];
public:
	bool EMA_established = false; // whether the EMA get enough numbers for initialization
	const int INIT_NUM = 20; // after INIT_NUM numbers are input, EMA are established
	int num_terms = 0;
	double weight = 0.9;
	std::vector<double> S_pre;
	
	double* filter(std::vector<double> angles);
	void feed(std::vector<double> angles);
};


#endif // !MOVING_AVERAGE_HEADER



