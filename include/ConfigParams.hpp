// 这个头文件包含了大部分配置参数
#ifndef _CONFIG_PARAMS_HEADER
#define _CONFIG_PARAMS_HEADER
// parameters for image acquisition
enum class triggerType
{
	SOFTWARE,
	HARDWARE
};
// 选择相机的触发模式，当连接有同步器时，选择HARDWARE即硬件触发
const triggerType chosenTrigger = triggerType::HARDWARE;
// 这是四个相机的采集范围相对于原点的偏置
static cv::Point2i offset[4] = { cv::Point(700, 780), cv::Point(700,560), cv::Point(700,750), cv::Point(650,550) };
// 四个相机采集范围的高与宽（依次）
static int64_t height[4] = { 900, 900, 900, 900 };
static int64_t width[4] = { 768, 768, 640, 640 }; // multiple of 32
// 在内存中缓存的图像数目，太大则占用内存，太小则影响同步处理
const int64_t numBuffers = 2;
// 想要设定的帧率（目前来说还无法正常工作）
const float frameRate = 80.0f;
// 曝光时间，以微秒为单位
static float exposureTimeToSet = 600.0f;
const bool SetExposureManual = true;
// 在标定世界坐标系时设为false
static bool TRACKING = true;

// 是否把中间处理过程的输出打印出来
const bool _PRINT_PROCESS = false;
// MAX and MIN H, S, V 
// for color thresholding
const cv::Scalar UPPER_RED = cv::Scalar(255, 255, 255);
const cv::Scalar LOWER_RED = cv::Scalar(230, 220, 220);
// number of cameras and markers
const int NUM_CAMERAS = 4;
const int NUM_MARKERS = 6;
const int NUM_MARKER_SET = 3;
const double weight = 0.5;
#endif // !_CONFIG_PARAMS_HEADER
