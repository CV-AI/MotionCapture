#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;

Mat img1, img2, gray1, gray2;
cv::Size imageSize(2048, 2048);

void load_image_points(int num_columns, int num_rows, int num_imgs, float square_size,
	char* upperimg_dir, char* lowerimg_dir) {
	
	object_points.clear();
	imagePoints1.clear();
	imagePoints2.clear();
	corners1.clear();
	corners2.clear();
	left_img_points.clear();
	right_img_points.clear();
	Size board_size = Size(num_columns, num_rows);
	int board_n = num_columns * num_rows;

	for (int i = 4; i < num_imgs; i++) {
		char left_img[100], right_img[100];
		sprintf(left_img, "%s%d.png", upperimg_dir, i);
		sprintf(right_img, "%s%d.png", lowerimg_dir, i);
		img1 = imread(left_img, cv::IMREAD_COLOR);
		img2 = imread(right_img, cv::IMREAD_COLOR);
		
		cvtColor(img1, gray1, cv::COLOR_RGB2GRAY);
		cvtColor(img2, gray2, cv::COLOR_RGB2GRAY);
		bool found1 = false, found2 = false;

		found1 = cv::findChessboardCorners(img1, board_size, corners1,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		found2 = cv::findChessboardCorners(img2, board_size, corners2,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);


		if (!found1 || !found2) {
			cout << "Chessboard find error!" << endl;
			cout << "leftImg: " << left_img << " and rightImg: " << right_img << endl;
			continue;
		}

		if (found1)
		{
			cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray1, board_size, corners1, found1);
		}
		if (found2)
		{
			cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
				cv::TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			cv::drawChessboardCorners(gray2, board_size, corners2, found2);
		}

		vector< Point3f > obj;
		for (int i = 0; i < num_rows; i++)
			for (int j = 0; j < num_columns; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found1 && found2) {
			cout << i << ". Found corners!" << endl;
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			object_points.push_back(obj);
		}
	}
	for (int i = 0; i < imagePoints1.size(); i++) {
		vector< Point2f > v1, v2;
		for (int j = 0; j < imagePoints1[i].size(); j++) {
			v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
			v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
		}
		left_img_points.push_back(v1);
		right_img_points.push_back(v2);
	}
}

int main(int argc, char const* argv[])
{
	char* out_file = "calib.yml";
	int num_imgs = 23;
	load_image_points(26,18, num_imgs, 20, "LeftUpper/", "LeftLower/");

	printf("Starting Calibration\n");
	cv::Mat D1, D2, R, T, E, F;
	cv::Mat M1 = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat M2 = cv::Mat::eye(3, 3, CV_64F);


	stereoCalibrate(object_points, left_img_points, right_img_points, M1, D1, M2, D2, img1.size(), R, T, E, F,
		cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
		cv::CALIB_SAME_FOCAL_LENGTH,
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
			1e-5));

	cv::FileStorage fs(out_file, cv::FileStorage::WRITE);
	/*fs << "M1" << M1;
	fs << "M2" << M2;
	fs << "D1" << D1;
	fs << "D2" << D2;
	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;*/

	printf("Done Calibration\n");

	printf("Starting Rectification\n");

	cv::Mat R1, R2, P1, P2, Q, map11, map12, map21, map22;

	stereoRectify(M1, D1, M2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
	cout << "Q left" << Q;
	// Precompute maps for cvRemap()
	initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_32FC2, map11,
		map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_32FC2, map21,
		map22);
	cv::Mat Map;
	cv::convertMaps(map11, map12, Map, cv::noArray(), CV_16SC2, true);
	fs << "Q0" << Q;
	fs << "map0" << Map;
	cv::convertMaps(map21, map22, Map, cv::noArray(), CV_16SC2, true);
	fs << "map1" << Map;
	printf("Done Rectification on Left Camera Set \n");

	load_image_points(26, 18, num_imgs, 20, "RightUpper/", "RightLower/");

	printf("Starting Calibration on Set 1\n");


	stereoCalibrate(object_points, left_img_points, right_img_points, M1, D1, M2, D2, img1.size(), R, T, E, F,
		cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST |
		cv::CALIB_SAME_FOCAL_LENGTH,
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100,
			1e-5));

	/*fs << "M1" << M1;
	fs << "M2" << M2;
	fs << "D1" << D1;
	fs << "D2" << D2;
	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;*/

	printf("Done Calibration\n");

	printf("Starting Rectification\n");


	stereoRectify(M1, D1, M2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
	cout << "Q right" << Q;
	// Precompute maps for cvRemap()
	initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_32FC2, map11,
		map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_32FC2, map21,
		map22);
	
	cv::convertMaps(map11, map12, Map, cv::noArray(), CV_16SC2, true);
	fs << "Q1" << Q;
	fs << "map2" << Map;
	cv::convertMaps(map21, map22, Map, cv::noArray(), CV_16SC2, true);
	fs << "map3" << Map;
	printf("Done Rectification on Right Camera Set \n");
	return 0;
}
