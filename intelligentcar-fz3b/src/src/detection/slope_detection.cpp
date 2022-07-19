#pragma once
/**
 * @file slope_detection.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 坡道（桥）路径处理
 * @version 0.1
 * @date 2022-06-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/predictor.hpp"
#include "../recognition/track_recognition.cpp"

using namespace cv;
using namespace std;

class SlopeDetection
{
public:
    bool slopeDetection(TrackRecognition &track, vector<PredictResult> predict)
    {
        if (slopEnable) //进入坡道
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2) //切行，防止错误前瞻引发转向
            {
                track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() / 2);
                track.pointsEdgeRight.resize(track.pointsEdgeRight.size() / 2);
            }
            counterSession++;
            if (counterSession > 30) //上桥40场图像后失效
            {
                counterRec = 0;
                counterSession = 0;
                slopEnable = false;
            }

            return true;
        }
        else //检测坡道
        {
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].label == LABEL_SLOP)
                {
                    counterRec++;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 3 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    slopEnable = true; //检测到坡道
                    return true;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }

            return false;
        }
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(TrackRecognition track, Mat &image)
    {
        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); //绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); //黄色点
        }

        if (slopEnable)
            putText(image, "SlopEnable", Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

private:
    uint16_t counterSession = 0; //图像场次计数器
    uint16_t counterRec = 0;     //加油站标志检测计数器
    bool slopEnable = false;     //坡道使能标志
};