#pragma once
#include <opencv2/opencv.hpp>
#include<iostream>
#include<vector>

#include<cmath>


class TemplateMatch
{
	void ColorThresholding(cv::Mat img_copy);
public:
 
	cv::Point detectWindowPosition;
	cv::Point minPoint;   //ģ��ƥ��result��Сֵλ��
	cv::Point maxPoint;    //ģ��ƥ��result���ֵλ��
	cv::Point init_template(int i);
	cv::Point update_template(int i);
	static cv::Point start_point[2][6]; //��ʼ֡ģ���е�����
	static cv::Point originalPoint; //���ο����
	static cv::Point processPoint; //���ο��յ�
	int predict_x[6];
	int predict_y[6];
	cv::Mat detectWindow;//��ⴰ��
	static cv::Mat image;   //��Ƶ��
	static cv::Mat imageCopy; //���ƾ��ο�ʱ��������ԭͼ��ͼ��
	cv::Mat rectImage;  //��ͼ��
	cv::Mat ImageResult;  //ģ��ƥ��result
	static cv::Mat image_l;
	static cv::Mat image_r;
	static bool leftButtonDownFlag; //�����������Ƶ��ͣ���ŵı�־λ

	bool getmomentpointEx = false;
	static bool gettempl_RIGHT;
	static bool gettempl_LEFT;
	bool auto_templ_left = false;
	bool auto_templ_right = false;
	static bool getColors;

	double minValue;   //ģ��ƥ��result��Сֵ
	double maxValude;   //ģ��ƥ��result���ֵ
	
	static std::vector<cv::Mat> Template_batch;

	int resultRows;  //ģ��ƥ��result����
	int resultcols;  //ģ��ƥ��result����
	int threshold = 130;
	static int index_template ;
	static int CorlorsChosen[3];//��ɫ���,˽�б��������Թ���
	static void Mouse_getColor(int event, int x, int y, int, void*);
	void auto_Template(int picture);// use color to get the first Template
	

};
