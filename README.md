# Motion capture

- 利用 CMake 构建
- 需要C++11 以上


## 一、 使用方法
### 生成
    - 使用cmake 构建项目

###   1. MotionCapture

    对标记点进行跟踪并输出经过低通滤波的步态角度
    其工作流程如下：
    - 自动生成相机标定矩阵，等待数十帧以保证白平衡完成（否则颜色会出现错误，导致识别标记点错误）
    - 尝试从build目录下的FrameDefine.yml获取相机坐标系到世界坐标系的转换矩阵，如果失败则转为从场景中的标定板自动生成转换矩阵并将之存放到FrameDefine.yml中
    - 为四个相机依次弹出detectwindow窗口（黑白），用户需要从窗口中选取标记点所在区域（以排除区域外的干扰），弹出窗口依次为左上、左下、右上、右下（从机器人面向相机支架来看）的相机。选取完成后，自动定位各个标记点并展开追踪。
    - 开始连续输出经过低通滤波的角度信息。这是一个长度为6的double 类型数组，依次为左髋、左膝、左踝、右髋、右膝、右踝的关节角（以度为单位）。

## 二、项目的实现
生产的解决方案中一共有三个项目
###   1. MotionCapture
    - 相机序号：以左上、左下、右上、右下（从机器人面向相机支架来看）分别为0，1，2，3号相机。
    - 跟踪算法是跟踪一对标记点（大腿、小腿、足）而非每个标记点都创建跟踪器

## 三、世界坐标系的变换
流程如下：

    - 获取棋盘格中三个点的空间位置（相机坐标系），以其中一点为原点，获得其他两点在此坐标系下的坐标p0, p1
    - 由p1, p0的叉乘得到p2，这个向量便是垂直于棋盘的向量（设为X轴），p0设为Y轴
    - 由p2， p0的叉乘得到X轴
    - 矩阵[X, Y, Z]便是由是世界坐标系到相机坐标系的旋转矩阵，设为R
    - 原点在相机坐标系中的位置便是世界坐标系到相机坐标系的位移向量, 设为T
    - 所以R.inv是相机坐标系到世界坐标系的旋转矩阵，由于旋转矩阵都是正交矩阵，所以R.inv == R.Transpose
    - 而-T便是相机坐标系到世界坐标系的位移向量
    - 故 P_world = R.inv*(P_camera-T) (矩阵乘法)
## 四、双目相机的标定
左腿和右腿的运动各有一对相机组成。
标定这些相机需要以下步骤

    - 收集相机对的同步采集的照片，可以通过项目RetriveImagePairs完成
    - 通过matlab中的stereo calibration工具箱标定双目相机
    - 保存左右两对双目相机的参数到两个不同的.mat文件
    - 通过save_camera_calib_to_yml.m 文件，将.mat文件转为opencv可以读取的.yml文件（即calib_params.yml，注意此文件包括了两对双目相机的参数）

## TODO

- 关于速度的问题
    - 整体速度取决于三个部分：1. 处理的速度； 2. 获取图像的速度（同步帧率）3. 前两者之间的同步问题