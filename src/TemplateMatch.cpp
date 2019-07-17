#include "include/TemplateMatch.h"
#include <numeric>
#define size_template 18


void TemplateMatch::ColorThresholding(cv::Mat img_copy)
{
	for (int j = 0; j < img_copy.rows; j++)
	{
		uchar*data = img_copy.ptr<uchar>(j);
		for (int i = 0; i < img_copy.cols; i++)
		{

			if ((abs(data[3 * i] - CorlorsChosen[0]) + abs(data[3 * i + 1] - CorlorsChosen[1]) + abs(data[3 * i + 2] - CorlorsChosen[2])) < threshold)
			{
				data[3 * i] = data[3 * i + 1] = data[3 * i + 2] = 255;
			}
			else data[3 * i] = data[3 * i + 1] = data[3 * i + 2] = 0;
		}
	}
	std::cout << "img_copy.channles():"<<img_copy.channels() << std::endl;
}

cv::Point TemplateMatch::init_template(int i)
{
	try
	{	
		ImageResult = cv::Mat::zeros(Template_batch[i].rows, Template_batch[i].cols, CV_8UC1);
		assert(Template_batch[i].channels() == image.channels());
		cv::matchTemplate(image, Template_batch[i], ImageResult,  CV_TM_SQDIFF);  //ƥ�䣬���ض��ƥ����
		std::cout << "Imageresult.channels(): "<<ImageResult.channels() << std::endl;
		cv::minMaxLoc(ImageResult, &minValue, &maxValude, &minPoint, &maxPoint, cv::Mat());  //ƥ��������ֵ���������ȡ
		//cv::rectangle(templ.image, templ.minPoint, cv::Point(templ.minPoint.x + templ.Template_batch[i].cols, templ.minPoint.y + templ.Template_batch[i].rows), cv::Scalar(0, 0, 255), 2);//��ͼ�л�������λ��				
		//Template_batch[i] = image(cv::Rect(maxPoint, cv::Point(maxPoint.x + Template_batch[i].cols,
			//			maxPoint.y + Template_batch[i].rows)));//���µ�ǰģ��ƥ���ģ��		

	}
	catch (const std::exception& )
	{
		printf("error\n");
	}
	return minPoint + cv::Point(size_template / 2, size_template / 2); //batch������ͼ���е��е�λ������
	
}

cv::Point TemplateMatch::update_template(int i) 
{


	//ע��detectwindow ��template_batch ��ͨ�����Ƿ���ͬ��������
	//cv::rectangle(templ.image, templ.minPoint, cv::Point(templ.minPoint.x + templ.Template_batch[i].cols, templ.minPoint.y + templ.Template_batch[i].rows), cv::Scalar(0, 0, 255), 2);//��ͼ�л�������λ��				
	// Template_batch[i] = image(cv::Rect(maxPoint, cv::Point(maxPoint.x + Template_batch[i].cols,
	//maxPoint.y + Template_batch[i].rows)));//���µ�ǰģ��ƥ���ģ�� 
	cv::Mat  img_copy = detectWindow.clone();
	std::vector<int> loc_x;
	std::vector<int> loc_y;
	for (int j = 0; j < img_copy.rows; j++)
	{
		uchar*data = img_copy.ptr<uchar>(j);
		for (int i = 0; i < img_copy.cols; i++)
		{

			if ((abs(data[3 * i] - CorlorsChosen[0]) + abs(data[3 * i + 1] - CorlorsChosen[1]) + abs(data[3 * i + 2] - CorlorsChosen[2])) < threshold)
			{
				loc_x.push_back(i);
				loc_y.push_back(j);
			}
		}
	}
	if (loc_x.size() > 4) //�����ʱ�̣�³�����Բ�ķ���ʧ�ܣ�����ú�ʱ��һ���ģ��ƥ��
	{
		minPoint = cv::Point(int(std::accumulate(loc_x.begin(), loc_x.end(), 0) / loc_x.size()),
			int(std::accumulate(loc_y.begin(), loc_y.end(), 0) / loc_y.size()));
		return minPoint;
	}
	else
	{
		cv::matchTemplate(detectWindow, Template_batch[i], ImageResult, CV_TM_SQDIFF);  //ƥ�䣬���ض��ƥ����
		cv::minMaxLoc(ImageResult, &minValue, &maxValude, &minPoint, &maxPoint, cv::Mat());  //ƥ��������ֵ���������ȡ
		return minPoint + cv::Point(size_template / 2, size_template / 2);
	}
	//����ڴ�
	std::vector<int>().swap(loc_x);
	std::vector<int>().swap(loc_y);
}



  void TemplateMatch::Mouse_getColor(int event, int x, int y, int, void*)
  {
	  static cv::Point origin;
	  static cv::Rect selection;//��̬�������Կ����ڴ棿
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
		  getColors = true;
		  std::cout << "Colour area has been selected!" << std::endl;

		  int SumOfChannelOneColor = 0, SumOfChannelTwoColor = 0;
		  int SumOfChannelThreeColor = 0, SumOfChannelFourColor = 0;
		  for (int j = selection.y; j < selection.y + selection.height; j++)
		  {
			  uchar*data = image_r.ptr<uchar>(j);
			  for (int i = selection.x; i < selection.x + selection.width; i++)
			  {
				  SumOfChannelOneColor += data[i * 3];
				  SumOfChannelTwoColor += data[i * 3 + 1];
				  SumOfChannelThreeColor += data[i * 3 + 2];
				  //SumOfChannelFourColor += data[i * 3 + 3];
			  }
		  }
		  int SumOfPixels = selection.width*selection.height;
		  CorlorsChosen[0] = static_cast<int>(SumOfChannelOneColor / SumOfPixels);
		  CorlorsChosen[1] = static_cast<int>(SumOfChannelTwoColor / SumOfPixels);
		  CorlorsChosen[2] = static_cast<int>(SumOfChannelThreeColor / SumOfPixels);
		  std::cout << "blue green red: " << CorlorsChosen[0] << " " << CorlorsChosen[1] << " " << CorlorsChosen[2] << std::endl;
	  }
 }
  void TemplateMatch::auto_Template(int picture)
  {
	  
		cv::Mat gray_image;
		gray_image = image.clone();
		ColorThresholding(gray_image);
		cv::Mat mask(3, 3, CV_8U, cv::Scalar(1));
		cv::morphologyEx(gray_image, gray_image, cv::MORPH_CLOSE, mask);

		std::vector<std::vector<cv::Point>>contours;
		cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
		assert(gray_image.channels() == 1);
		cv::findContours(gray_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		int cmin = 20; //contour����Сֵ
		int cmax = 120;
		std::vector<std::vector<cv::Point>>::const_iterator itc = contours.begin();
		while (itc != contours.end())
		{
			//std::cout << "size: " << itc->size() << std::endl;
			if (itc->size() < cmin || itc->size() > cmax)
			{
				itc = contours.erase(itc);//��ȥ������Ҫ��ĵ�����
			}
			else itc++;
		}
		if (contours.size() == 6)
		{
			std::vector<std::vector<cv::Point>>::const_iterator it = contours.begin();

			int i = 0;
			while (it != contours.end())
			{
				cv::Moments mom = cv::moments(cv::Mat(*it++));
				start_point[picture][i] = cv::Point(mom.m10 / mom.m00, mom.m01 / mom.m00);	
				//cv::circle(image, start_point[picture][i], 2, cv::Scalar(0, 255, 0), 2);
				i++;
			}
			if (start_point[picture][0].x > start_point[picture][1].x)//��֤�ż������·���������Ͳɼ���������λ���й�
			{
				cv::Point tempP;
				tempP = start_point[picture][0];
				start_point[picture][0] = start_point[picture][1];
				start_point[picture][1] = tempP;
			}
			for (int i = 0; i < 6; i++)//һ��Ҫ��start_point��������֮����ܳ�ʼ��Template_batch
			{
				Template_batch[picture * 6 + i] = image(cv::Rect(start_point[picture][i] - cv::Point(size_template / 2, size_template / 2),
					start_point[picture][i] + cv::Point(size_template / 2, size_template / 2)));
			}
			if (picture == 0)
			{
				auto_templ_left = true;
			}
			if (picture == 1)
			{
				auto_templ_right = true;
			}
			
		}

		else
		{
			std::cout << "Contours numbers are wrong��the number of contours is " << contours.size() << std::endl; 
			return;
		}
  }


// �����ʼ����̬�����������ڴ�
std::vector<cv::Mat> TemplateMatch::Template_batch(12);
bool TemplateMatch::gettempl_RIGHT = false;
bool TemplateMatch::gettempl_LEFT = false;
bool TemplateMatch::leftButtonDownFlag = false;
cv::Point TemplateMatch::originalPoint = cv::Point(0, 0);
cv::Point TemplateMatch::processPoint = cv::Point(0, 0);
cv::Point TemplateMatch::start_point[2][6] = { {cv::Point(0, 0),cv::Point(0, 0),cv::Point(0, 0) ,cv::Point(0, 0) ,cv::Point(0, 0) ,cv::Point(0, 0) },
											{cv::Point(0, 0),cv::Point(0, 0),cv::Point(0, 0) ,cv::Point(0, 0) ,cv::Point(0, 0) ,cv::Point(0, 0) } };

cv::Mat TemplateMatch::image_l = cv::Mat::zeros(4, 5, CV_8UC3);//������ʼ���������ģ��
cv::Mat TemplateMatch::image_r = cv::Mat::zeros(4, 5, CV_8UC3);
int TemplateMatch::index_template = 0;
cv::Mat TemplateMatch::image = cv::Mat::zeros(4, 5, CV_8UC3);
cv::Mat TemplateMatch::imageCopy = cv::Mat::zeros(4, 5, CV_8UC3);
bool TemplateMatch::getColors = false;
int TemplateMatch::CorlorsChosen[3];