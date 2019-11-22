#ifndef _CONFIG_PARAMS_HEADER
#define _CONFIG_PARAMS_HEADER
// parameters for image acquisition
enum triggerType
{
	SOFTWARE,
	HARDWARE
};
const triggerType chosenTrigger = HARDWARE;
static cv::Point2i offset[4] = { cv::Point(500, 750), cv::Point(500,550), cv::Point(780,750), cv::Point(800,550) };
const int64_t numBuffers = 2;
const float frameRate = 80.0f;
static float exposureTimeToSet = 12000.0f;
static int64_t height[4] = { 900, 900, 900, 900 };
static int64_t width[4] = { 768, 768, 640, 640 }; // multiple of 32
static bool SetExposureManual = true;
static bool TRACKING = true;


// MAX and MIN H, S, V
const int MAX_H_RED = 20;
const int MIN_H_RED = 0;
const int MAX_SATURATION = 220;
const int MIN_SATURATION = 120;
const int MAX_VALUE = 255;
const int MIN_VALUE = 90;
static const int NUM_CAMERAS = 4;
static const int NUM_MARKERS = 6;
static const int NUM_MARKER_SET = 3;
const double weight = 0.75;
#endif // !_CONFIG_PARAMS_HEADER
