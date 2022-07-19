#pragma once
/**
 * @file image_preprocess.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 图像预处理：RGB转灰度图，图像二值化
 * @version 0.1
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace cv;
using namespace std;

/**
**[1] 读取视频
**[2] 图像二值化
*/

class ImagePreprocess
{
public:
	/**
	 * @brief 图像二值化
	 *
	 * @param frame	输入原始帧
	 * @return Mat	二值化图像
	 */
	Mat imageBinaryzation(Mat &frame)
	{
		Mat imageGray, imageBinary;

		cvtColor(frame, imageGray, COLOR_BGR2GRAY); // RGB转灰度图

		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

		return imageBinary;
	}

private:
	bool correctionEnable = false; //图像矫正使能：初始化完成
	Mat cameraMatrix;			   // 摄像机内参矩阵
	Mat distCoeffs;				   // 相机的畸变矩阵
};
