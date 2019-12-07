#ifndef _CONFIG_PARAMS_HEADER
#define _CONFIG_PARAMS_HEADER
// parameters for image acquisition
enum class triggerType
{
	SOFTWARE,
	HARDWARE
};
const triggerType chosenTrigger = triggerType::HARDWARE;
static cv::Point2i offset[4] = { cv::Point(500, 780), cv::Point(500,560), cv::Point(900,750), cv::Point(950,550) };
static int64_t height[4] = { 900, 900, 900, 900 };
static int64_t width[4] = { 768, 768, 640, 640 }; // multiple of 32
const int64_t numBuffers = 2;
const float frameRate = 80.0f;
static float exposureTimeToSet = 8000.0f;

const bool SetExposureManual = true;
// ��ֻ��Ҫ��ȡͼ��ʱ����Ϊfalse
static bool TRACKING = true;

// �Ƿ��ӡ�м������Ϣ����ʽ����ʱ��Ϊfalse�����и���
const bool _PRINT_PROCESS = false;
// MAX and MIN H, S, V
const int MAX_H_RED = 15;
const int MIN_H_RED = 0;
const int MAX_SATURATION = 15;
const int MIN_SATURATION = 0;
const int MAX_VALUE = 255;
const int MIN_VALUE = 220;
const const int NUM_CAMERAS = 4;
const const int NUM_MARKERS = 6;
const const int NUM_MARKER_SET = 3;
const double weight = 0.5;
#endif // !_CONFIG_PARAMS_HEADER
