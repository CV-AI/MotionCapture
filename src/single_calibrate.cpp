#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <fstream>
using namespace std;
using namespace cv;
#define PATH "/camera_calibration/image_my/"
#define NUM 30
int main() 
{
    // 定义用来保存导入的图片
    Mat image_in; 
    // 定义用来保存文件路径的容器
    vector<string> filelist;
    // 定义用来保存旋转和平移矩阵的容器
    vector<Mat> rvecs, tvecs;
    // 定义相机矩阵，畸变矩阵
    Mat cameraMatrix;
    Mat distCoeffs;
    int flags = 0;
    // 定义保存图像二维角点的容器
    vector<Point2f> corners;
    // 定义保存图像三维角点的容器
    vector<vector<Point2f> > corners2;
    // 定义保存图像二维和三维角点的容器
    vector<Point3f> worldPoints;
    vector<vector<Point3f> > worldPoints2;
    //***************读取一个文件夹中的所有图片（所有标定图片）**********************
    for(int i=1; i<NUM;i++) 
    {
        stringstream str;
        str << PATH << setw(2) << setfill('0') << i << ".jpg";
        // 保存所有图片的路径，放入容器filelist中
        filelist.push_back(str.str());
        image_in = imread(str.str());
    }
}