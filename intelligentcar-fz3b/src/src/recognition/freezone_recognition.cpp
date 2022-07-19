#pragma once
/**
 * @file freezone_recognition.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 泛行区识别与路径规划
 * @version 0.1
 * @date 2022-03-30
 * @copyright Copyright (c) 2022
 * @note 泛行区处理步骤：
 *                      [1] 识别泛行区入口标志（等边三角形检测）
 *                      [2] 赛道补偿点搜索&路径处理
 *                      [3] 入泛行区完成
 *                      [4] 识别泛行区出口标志（等边三角形检测）
 *                      [5] 赛道补偿点搜索&路径处理
 *                      [6] 出泛行区完成
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

class FreezoneRecognition
{
public:
    enum FreezoneStep
    {
        None = 0,            //离开泛行区
        FreezoneEntering,    //入泛行区
        FreezoneEnterFinish, //入泛行区完成
        FreezoneExiting,     //出泛行区
    };
    FreezoneStep freezoneStep = FreezoneStep::None; //泛行区状态
    uint16_t counterTriChecked = 0;                 //等边三角形连续检测计数器
    bool directionLeft = true;

    /**
     * @brief 泛行区识别初始化
     *
     */
    void reset(void)
    {
        freezoneStep = FreezoneStep::None; //泛行区行使状态
        counterSession = 0;                //图像场次计数器
        counterRec = 0;                    //标志检测计数器
        counterStep = 0;
    }

    /**
     * @brief 泛型区识别与路径规划
     *
     * @param track 赛道识别结果
     * @param ipm 透视变换类
     */
    bool freezoneRecognition(TrackRecognition &track, vector<PredictResult> predict)
    {
        _pointL = POINT(0, 0);
        _pointR = POINT(0, 0);
        _spurroad = POINT(0, 0);
        peakTriangleIpm.clear();
        uint16_t rowBreakLeft = 0;  //赛道左边缘补偿点
        uint16_t rowBreakRight = 0; //赛道右边缘补偿点
        _index = "";

        // [01] 泛型区入口标志识别
        if (track.spurroad.size() >= 1) //岔路标志
        {
            int indexSpurroad = track.spurroad.size() - 1;
            // [01] 泛型区入口标志识别
            if (freezoneStep == FreezoneStep::None || freezoneStep == FreezoneStep::FreezoneEnterFinish) //入泛行区与出泛行区标志识别
            {

                POINT freezone = searchFreezoneSign(predict);
                if (freezone.x > 0)
                    counterRec++;

                if (counterRec)
                {
                    counterSession++;
                    if (counterRec >= 3 && counterSession < 8)
                    {
                        freezoneStep = FreezoneStep::FreezoneEntering; //使能
                        counterRec = 0;
                        counterSession = 0;
                    }
                    else if (counterSession >= 8)
                    {
                        counterRec = 0;
                        counterSession = 0;
                    }
                }

                _index = "1";
                rowBreakLeft = searchBreakLeft(track.pointsEdgeLeft);
                rowBreakRight = searchBreakRight(track.pointsEdgeRight);

                // for (int i = 0; i < track.spurroad.size(); i++)
                // {
                //     if (track.spurroad[i].x < track.pointsEdgeRight[rowBreakRight].x && track.spurroad[i].x < track.pointsEdgeLeft[rowBreakLeft].x)
                //     {
                //         indexSpurroad = i;
                //         _spurroad = track.spurroad[i];
                //         break;
                //     }
                // }

                if (track.spurroad[indexSpurroad].x < track.pointsEdgeRight[rowBreakRight].x && track.spurroad[indexSpurroad].x < track.pointsEdgeLeft[rowBreakLeft].x)
                {
                    //等边三角形边长计算
                    vector<POINT> peaks;
                    peaks.push_back(track.pointsEdgeLeft[rowBreakLeft]);
                    peaks.push_back(track.spurroad[indexSpurroad]);
                    peaks.push_back(track.pointsEdgeRight[rowBreakRight]);

                    if (freezoneStep == FreezoneStep::None) //入泛行区
                    {
                        if (regularTriangleCheck(peaks) < 10.0) //连续检测到等边三角形
                        {
                            counterTriChecked++;
                            if (counterTriChecked > 2)
                            {
                                freezoneStep = FreezoneStep::FreezoneEntering;
                                counterTriChecked = 0;
                                counterStep = 0;
                            }
                        }
                        else
                        {
                            counterTriChecked = 0;
                        }
                    }
                    else if (freezoneStep == FreezoneStep::FreezoneEnterFinish) //出泛行区
                    {
                        if (regularTriangleCheck(peaks) < 30.0) //连续检测到等边三角形
                        {
                            counterTriChecked++;
                            if (counterTriChecked > 2)
                            {
                                freezoneStep = FreezoneStep::FreezoneExiting;
                                counterTriChecked = 0;
                                counterStep = 0;
                            }
                        }
                        else
                            counterTriChecked = 0;
                    }
                }
            }

            //[02] 入泛型区：赛道路径重构
            if (freezoneStep == FreezoneStep::FreezoneEntering || freezoneStep == FreezoneStep::FreezoneExiting)
            {
                if (rowBreakLeft == 0)
                    rowBreakLeft = searchBreakLeft(track.pointsEdgeLeft);
                if (rowBreakRight == 0)
                    rowBreakRight = searchBreakRight(track.pointsEdgeRight);

                if (directionLeft) //选择左入泛行区
                {
                    bool repairEnable = false;
                    _index = "2";
                    if (track.pointsEdgeRight[rowBreakRight].x > track.spurroad[indexSpurroad].x)
                    {
                        _index = "3";
                        repairEnable = true;
                    }
                    else if (track.pointsEdgeRight[rowBreakRight].x <= track.spurroad[indexSpurroad].x &&
                             track.pointsEdgeLeft[rowBreakLeft].x > track.spurroad[indexSpurroad].x) //未搜索到右边缘补偿点：坐标高于岔路坐标
                    {
                        if (rowBreakLeft < track.pointsEdgeRight.size()) //由左边缘补偿点映射右边缘补偿点
                        {
                            rowBreakRight = rowBreakLeft;
                            if (track.spurroad[indexSpurroad].x != track.pointsEdgeRight[rowBreakRight].x)
                            {
                                _index = "4";
                                repairEnable = true;
                            }
                        }
                    }
                    else
                    {
                        rowBreakRight = 0;
                        if (track.spurroad[indexSpurroad].x != track.pointsEdgeRight[rowBreakRight].x)
                        {
                            _index = "5";
                            repairEnable = true;
                        }
                    }

                    if (repairEnable) //开始补偿赛道边缘
                    {
                        uint16_t rowEnd = rowBreakRight;
                        for (int i = rowBreakRight; i < track.pointsEdgeRight.size(); i++)
                        {
                            if (track.pointsEdgeRight[i].x <= track.spurroad[indexSpurroad].x)
                            {
                                rowEnd = i;
                                break;
                            }
                        }
                        if (rowEnd != rowBreakRight)
                        {
                            double k = (double)(track.spurroad[indexSpurroad].y - track.pointsEdgeRight[rowBreakRight].y) /
                                       (double)(track.spurroad[indexSpurroad].x - track.pointsEdgeRight[rowBreakRight].x);
                            double b = track.spurroad[indexSpurroad].y - k * track.spurroad[indexSpurroad].x;
                            for (int i = rowBreakRight; i <= rowEnd; i++)
                            {
                                track.pointsEdgeRight[i].y = (int)(k * track.pointsEdgeRight[i].x + b);
                            }
                        }
                        track.trackRecognition(true, rowEnd); //赛道边缘重新搜索
                    }
                }
            }

            _pointL = track.pointsEdgeLeft[rowBreakLeft];
            _pointR = track.pointsEdgeRight[rowBreakRight];
        }
        else if (track.spurroad.size() < 1) //未检测到岔路
        {
            _index = "6";
            if (freezoneStep == FreezoneStep::FreezoneEntering)
            {
                counterStep++;
                if (counterStep > 2)
                {
                    freezoneStep = FreezoneStep::FreezoneEnterFinish;
                    counterStep = 0;
                }
            }
            else if (freezoneStep == FreezoneStep::FreezoneExiting)
            {
                counterStep++;
                if (counterStep > 2)
                {
                    freezoneStep = FreezoneStep::None;
                    counterStep = 0;
                }
            }
        }

        //返回识别结果
        if (freezoneStep == FreezoneStep::FreezoneEntering || freezoneStep == FreezoneStep::FreezoneExiting)
            return true;
        else
            return false;
    }

    /**
     * @brief 等边三角形检验
     *
     * @param peaks 输入三角形三个顶点坐标
     * @param ipm 透视变换参数
     * @return sigma 边长方差
     */
    double regularTriangleCheck(vector<POINT> peaks)
    {
        if (peaks.size() != 3)
            return false;

        for (size_t i = 0; i < peaks.size(); i++) //坐标透视变换
        {
            Point2d peak = Point2d(peaks[i].y, peaks[i].x);
            peak = ipm.homography(peak);
            peaks[i] = POINT(peak.y, peak.x);
        }
        peakTriangleIpm = peaks;

        int vectorX, vectorY;
        vector<int> length;
        vectorX = pow(peaks[0].x - peaks[1].x, 2);
        vectorY = pow(peaks[0].y - peaks[1].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //边长：A

        vectorX = pow(peaks[1].x - peaks[2].x, 2);
        vectorY = pow(peaks[1].y - peaks[2].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //边长：B

        vectorX = pow(peaks[2].x - peaks[0].x, 2);
        vectorY = pow(peaks[2].y - peaks[0].y, 2);
        length.push_back(sqrt(vectorX + vectorY)); //边长：C

        double _sigma = sigma(length);
        cout << "lengthA:" << length[0] << " | lengthB:" << length[1] << " | lengthC:" << length[2] << " | sigma:" << _sigma << endl;

        return _sigma;
    }

    /**
     * @brief 绘制泛型区识别结果
     *
     * @param Image 需要叠加显示的图像/RGB
     * @param imageIpm 需要叠加的俯视域图像/RGB
     */
    void drawImage(TrackRecognition track, Mat &Image)
    {
        //绘制边缘点
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
                   Scalar(0, 255, 0), -1); //绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(Image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
                   Scalar(0, 255, 255), -1); //黄色点
        }

        //绘制岔路点
        // for (int i = 0; i < track.spurroad.size(); i++)
        // {
        //     circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 4,
        //            Scalar(0, 0, 255), -1); //红色点
        // }
        if (_spurroad.x > 0)
            circle(Image, Point(_spurroad.y, _spurroad.x), 4, Scalar(0, 0, 255), -1); //红色点

        //绘制左右边缘拐点
        circle(Image, Point(_pointL.y, _pointL.x), 5, Scalar(226, 43, 138), -1); //左补线点：紫色
        circle(Image, Point(_pointR.y, _pointR.x), 5, Scalar(226, 43, 138), -1); //右补线点：粉色

        // //绘制等腰三角形
        // if (peakTriangleIpm.size() == 3)
        // {
        //     circle(imageIpm, Point(peakTriangleIpm[0].y, peakTriangleIpm[0].x), 5, Scalar(226, 43, 138), -1);
        //     circle(imageIpm, Point(peakTriangleIpm[1].y, peakTriangleIpm[1].x), 5, Scalar(226, 43, 138), -1);
        //     circle(imageIpm, Point(peakTriangleIpm[2].y, peakTriangleIpm[2].x), 5, Scalar(226, 43, 138), -1);
        //     line(imageIpm, Point(peakTriangleIpm[0].y, peakTriangleIpm[0].x), Point(peakTriangleIpm[1].y, peakTriangleIpm[1].x), Scalar(0, 0, 255), 1);
        //     line(imageIpm, Point(peakTriangleIpm[0].y, peakTriangleIpm[0].x), Point(peakTriangleIpm[2].y, peakTriangleIpm[2].x), Scalar(0, 0, 255), 1);
        //     line(imageIpm, Point(peakTriangleIpm[2].y, peakTriangleIpm[2].x), Point(peakTriangleIpm[1].y, peakTriangleIpm[1].x), Scalar(0, 0, 255), 1);
        // }

        int type = (int)freezoneStep;
        putText(Image, _index + " | " + to_string(type), Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

        putText(Image, "Freezone", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
    }

private:
    POINT _pointL;
    POINT _pointR;
    string _index = "";
    vector<POINT> peakTriangleIpm; //三角形的顶点(IPM)
    uint16_t counterStep = 0;      //步骤切换计数器
    POINT _spurroad;
    uint16_t counterSession = 0; //图像场次计数器
    uint16_t counterRec = 0;     //标志检测计数器

    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeft(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeft = 0;
        uint16_t counter = 0;
        uint16_t counterBottom = 0; //底行过滤计数器

        for (int i = 0; i < pointsEdgeLeft.size() - 20; i++) //寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y < 20)
                counterBottom++;
            if (counterBottom > 3)
            {
                if (pointsEdgeLeft[i].y >= pointsEdgeLeft[rowBreakLeft].y)
                {
                    rowBreakLeft = i;
                    counter = 0;
                }
                else if (pointsEdgeLeft[i].y < pointsEdgeLeft[rowBreakLeft].y) //突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakLeft;
                }
            }
        }

        return rowBreakLeft;
    }

    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRight(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRight = 0;
        uint16_t counter = 0;
        uint16_t counterBottom = 0; //底行过滤计数器

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) //寻找左边跳变点
        {
            if (pointsEdgeRight[i].y > COLSIMAGE - 20)
                counterBottom++;
            if (counterBottom > 3)
            {
                if (pointsEdgeRight[i].y <= pointsEdgeRight[rowBreakRight].y)
                {
                    rowBreakRight = i;
                    counter = 0;
                }
                else if (pointsEdgeRight[i].y > pointsEdgeRight[rowBreakRight].y) //突变点计数
                {
                    counter++;
                    if (counter > 5)
                        return rowBreakRight;
                }
            }
        }

        return rowBreakRight;
    }

    /**
     * @brief 搜索泛行区标志
     *
     * @param predict
     * @return POINT
     */
    POINT searchFreezoneSign(vector<PredictResult> predict)
    {
        POINT freezone(0, 0);
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_FREEZONE) //标志检测
            {
                return POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2);
            }
        }

        return freezone;
    }
};
