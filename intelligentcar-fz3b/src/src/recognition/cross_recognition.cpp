#pragma once
/**
 * @file cross_recognition.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 十字道路识别与图像处理
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 * @note 十字道路处理步骤：
 *                      [01] 入十字类型识别：track_recognition.cpp
 *                      [02] 补线起止点搜索
 *                      [03] 边缘重计算
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"

using namespace cv;
using namespace std;

class CrossroadRecognition
{
public:
    /**
     * @brief 搜索十字赛道突变行（左上）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 5;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeLeft.size() - 5; i > 50; i--)
        {
            if (pointsEdgeLeft[i].y > 2 & abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 3)
            {
                rowBreakLeftUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeLeft[i].y <= 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeftUp;
            }
        }

        return rowBreakLeftUp;
    }
    /**
     * @brief 搜索十字赛道突变行（左下）
     *
     * @param pointsEdgeLeft
     * @return uint16_t
     */
    uint16_t searchBreakLeftDown(vector<POINT> pointsEdgeLeft)
    {
        uint16_t rowBreakLeftUp = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeLeft.size() - 10; i++) //寻找左边跳变点
        {
            if (pointsEdgeLeft[i].y > pointsEdgeLeft[rowBreakLeftUp].y)
            {
                rowBreakLeftUp = i;
                counter = 0;
            }
            else if (pointsEdgeLeft[i].y < pointsEdgeLeft[rowBreakLeftUp].y) //突变点计数
            {
                counter++;
                if (counter > 5)
                    return rowBreakLeftUp;
            }
        }

        return rowBreakLeftUp;
    }
    /**
     * @brief 搜索十字赛道突变行（右上）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightUp(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightUp = pointsEdgeRight.size() - 5;
        uint16_t counter = 0;
        uint16_t counterFilter = 0;
        for (int i = pointsEdgeRight.size() - 5; i > 50; i--)
        {
            if (pointsEdgeRight[i].y < COLSIMAGE - 2 & abs(pointsEdgeRight[i].y - pointsEdgeRight[i + 1].y) < 3)
            {
                rowBreakRightUp = i;
                counter = 0;
                counterFilter++;
            }
            else if (pointsEdgeRight[i].y >= COLSIMAGE - 2 && counterFilter > 10)
            {
                counter++;
                if (counter > 5)
                    return rowBreakRightUp;
            }
        }

        return rowBreakRightUp;
    }
    /**
     * @brief 搜索十字赛道突变行（右下）
     *
     * @param pointsEdgeRight
     * @return uint16_t
     */
    uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
    {
        uint16_t rowBreakRightDown = 0;
        uint16_t counter = 0;

        for (int i = 0; i < pointsEdgeRight.size() - 10; i++) //寻找左边跳变点
        {
            if (pointsEdgeRight[i].y < pointsEdgeRight[rowBreakRightDown].y)
            {
                rowBreakRightDown = i;
                counter = 0;
            }
            else if (pointsEdgeRight[i].y > pointsEdgeRight[rowBreakRightDown].y) //突变点计数
            {
                counter++;
                if (counter > 5)
                    return rowBreakRightDown;
            }
        }

        return rowBreakRightDown;
    }

    /**
     * @brief 直入十字搜索
     *
     * @param pointsEdgeLeft 赛道左边缘点集
     * @param pointsEdgeRight 赛道右边缘点集
     * @return true
     * @return false
     */
    bool searchStraightCrossroad(vector<POINT> pointsEdgeLeft, vector<POINT> pointsEdgeRight)
    {
        if (pointsEdgeLeft.size() < ROWSIMAGE * 0.8 || pointsEdgeRight.size() < ROWSIMAGE * 0.8)
        {
            return false;
        }

        uint16_t counterLeft = 0;
        uint16_t counterRight = 0;
        for (int i = pointsEdgeLeft.size() - 10; i > 1; i--) //搜索上半部分边缘点
        {
            if (pointsEdgeLeft[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeLeft[i].y < 2)
                counterLeft++;
        }
        for (int i = pointsEdgeRight.size() - 10; i > 1; i--) //搜索上半部分边缘点
        {
            if (pointsEdgeRight[i].x > ROWSIMAGE / 2)
                break;
            else if (pointsEdgeRight[i].y > COLSIMAGE - 2)
                counterRight++;
        }
        if (counterLeft > 30 && counterRight > 30)
            return true;
        else
            return false;
    }

    /**
     * @brief 十字道路识别与图像处理
     *
     * @param track 赛道识别结果
     * @param imagePath 输入图像
     */
    bool crossroadRecognition(TrackRecognition &track)
    {
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 || track.pointsEdgeRight.size() < ROWSIMAGE / 2)
            return false;

        if (track.stdevLeft < 100 && track.stdevRight < 100)
            return false;
        bool res = false;                                            //返回结果
        int ch = 0;                                                  //计数器
        double k = 0, b = 0;                                         //直线斜率
        int counterHigh = 0;                                         //计数器
        uint16_t rowBreakLeftUp = track.pointsEdgeLeft.size() - 1;   //十字突变行（左上）
        uint16_t rowBreakLeftDown = 0;                               //十字突变行（左下）
        uint16_t rowBreakRightUp = track.pointsEdgeRight.size() - 1; //十字突变行（右上）
        uint16_t rowBreakRightDown = 0;                              //十字突变行（右下）
        uint16_t counterBreakLeft = 0;                               //寻找左边突变点计数器
        uint16_t counterBreakRight = 0;                              //寻找右边突变点计数器
        repairRightEnable = false;                                   //右边缘使能标志
        repairLeftEnable = false;                                    //左边缘使能标志
        crossroadType = CrossroadType::None;                         //十字道路类型

        _pointLU = POINT(0, 0);
        _pointLD = POINT(0, 0);
        _pointRU = POINT(0, 0);
        _pointRD = POINT(0, 0);
        _index = 0;

        if (track.crossroadSlant.y == 1)
            crossroadType = CrossroadType::CrossroadLeft;
        else if (track.crossroadSlant.y == 2)
            crossroadType = CrossroadType::CrossroadRight;
        else if (track.crossroadSlant.y == 3)
            crossroadType = CrossroadType::CrossroadStraight;

        if (track.crossroadSlant.x > 0 && track.spurroad.size() > 0 && track.spurroad.size() < 3) //斜入十字使能
        {
            if (crossroadType == CrossroadType::CrossroadLeft) //左入十字
            {
                uint16_t rowStartRepair = 0;
                for (int i = 0; i < track.pointsEdgeRight.size() - 10; i++)
                {
                    if (track.pointsEdgeRight[i].y < COLSIMAGE - 2)
                    {
                        rowStartRepair = i;
                        break;
                    }
                }
                rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight); //搜索十字赛道突变行（右下）
                _pointRU = track.pointsEdgeRight[rowStartRepair];
                _pointRD = track.pointsEdgeRight[rowBreakRightDown];
                if (track.pointsEdgeRight[rowBreakRightDown].y > 0 && track.pointsEdgeRight.size() > track.crossroadSlant.x && track.pointsEdgeRight.size() > 5 && rowBreakRightDown > rowStartRepair)
                {
                    if (track.pointsEdgeRight[rowBreakRightDown].x != track.pointsEdgeRight[rowStartRepair].x)
                    {
                        k = (float)(track.pointsEdgeRight[rowBreakRightDown].y - track.pointsEdgeRight[rowStartRepair].y) / (float)(track.pointsEdgeRight[rowBreakRightDown].x - track.pointsEdgeRight[rowStartRepair].x);
                        b = track.pointsEdgeRight[rowBreakRightDown].y - k * track.pointsEdgeRight[rowBreakRightDown].x;
                        for (int i = track.pointsEdgeRight.size() - 1; i >= rowBreakRightDown; i--)
                        {
                            track.pointsEdgeRight[i].y = (int)(k * track.pointsEdgeRight[i].x + b);
                        }
                    }
                }

                //重新验证左边丢失赛道边缘
                rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft); //搜索十字赛道突变行（左下）
                _pointLD = track.pointsEdgeLeft[rowBreakLeftDown];
                if (abs(rowBreakLeftDown - rowBreakRightDown) < 30) //左右补线行差距较小
                {
                    for (int i = 0; i < track.pointsEdgeLeft.size() - 60; i++)
                    {
                        if (track.pointsEdgeLeft[i].y >= 2)
                        {
                            rowStartRepair = i;
                            break;
                        }
                    }
                    _pointLU = track.pointsEdgeLeft[rowStartRepair];

                    if (track.pointsEdgeLeft[rowBreakLeftDown].y > 0 && track.pointsEdgeLeft.size() > track.crossroadSlant.x && track.pointsEdgeLeft.size() > 5 && rowBreakLeftDown > rowStartRepair)
                    {
                        if (track.pointsEdgeLeft[rowBreakLeftDown].x != track.pointsEdgeLeft[rowStartRepair].x)
                        {
                            k = (float)(track.pointsEdgeLeft[rowBreakLeftDown].y - track.pointsEdgeLeft[rowStartRepair].y) / (float)(track.pointsEdgeLeft[rowBreakLeftDown].x - track.pointsEdgeLeft[rowStartRepair].x);
                            b = track.pointsEdgeLeft[rowBreakLeftDown].y - k * track.pointsEdgeLeft[rowBreakLeftDown].x;
                            for (int i = track.pointsEdgeLeft.size() - 1; i > rowBreakLeftDown; i--)
                            {
                                track.pointsEdgeLeft[i].y = (int)(k * track.pointsEdgeLeft[i].x + b);
                            }
                        }
                    }
                    // crossroadType = CrossroadType::CrossroadStraight;
                }

                track.pointsEdgeLeft.resize(track.crossroadSlant.x);  //切除顶部错误路径
                track.pointsEdgeRight.resize(track.crossroadSlant.x); //切除顶部错误路径

                res = true; //返回十字识别结果
            }
            if (crossroadType == CrossroadType::CrossroadRight) //右入十字
            {
                uint16_t rowStartRepair = 0;
                for (int i = 0; i < track.pointsEdgeLeft.size() - 60; i++)
                {
                    if (track.pointsEdgeLeft[i].y >= 2)
                    {
                        rowStartRepair = i;
                        break;
                    }
                }
                rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft); //搜索十字赛道突变行（左下）
                _pointLU = track.pointsEdgeLeft[rowStartRepair];
                _pointLD = track.pointsEdgeLeft[rowBreakLeftDown];
                if (track.pointsEdgeLeft[rowBreakLeftDown].y > 0 && track.pointsEdgeLeft.size() > track.crossroadSlant.x && track.pointsEdgeLeft.size() > 5 && rowBreakLeftDown > rowStartRepair)
                {
                    if (track.pointsEdgeLeft[rowBreakLeftDown].x != track.pointsEdgeLeft[rowStartRepair].x)
                    {
                        k = (float)(track.pointsEdgeLeft[rowBreakLeftDown].y - track.pointsEdgeLeft[rowStartRepair].y) / (float)(track.pointsEdgeLeft[rowBreakLeftDown].x - track.pointsEdgeLeft[rowStartRepair].x);
                        b = track.pointsEdgeLeft[rowBreakLeftDown].y - k * track.pointsEdgeLeft[rowBreakLeftDown].x;
                        for (int i = track.pointsEdgeLeft.size() - 1; i > rowBreakLeftDown; i--)
                        {
                            track.pointsEdgeLeft[i].y = (int)(k * track.pointsEdgeLeft[i].x + b);
                        }
                    }
                }

                //重新验证左边丢失赛道边缘
                rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight); //搜索十字赛道突变行（右下）
                _pointRD = track.pointsEdgeRight[rowBreakRightDown];
                if (abs(rowBreakRightDown - rowBreakLeftDown) < 30) //左右补线行差距较小
                {
                    for (int i = 0; i < track.pointsEdgeRight.size() - 10; i++)
                    {
                        if (track.pointsEdgeRight[i].y < COLSIMAGE - 2)
                        {
                            rowStartRepair = i;
                            break;
                        }
                    }
                    _pointRU = track.pointsEdgeRight[rowStartRepair];
                    if (track.pointsEdgeRight[rowBreakRightDown].y > 0 && track.pointsEdgeRight.size() > track.crossroadSlant.x && track.pointsEdgeRight.size() > 5 && rowBreakRightDown > rowStartRepair)
                    {
                        if (track.pointsEdgeRight[rowBreakRightDown].x != track.pointsEdgeRight[rowStartRepair].x)
                        {
                            k = (float)(track.pointsEdgeRight[rowBreakRightDown].y - track.pointsEdgeRight[rowStartRepair].y) / (float)(track.pointsEdgeRight[rowBreakRightDown].x - track.pointsEdgeRight[rowStartRepair].x);
                            b = track.pointsEdgeRight[rowBreakRightDown].y - k * track.pointsEdgeRight[rowBreakRightDown].x;
                            for (int i = track.pointsEdgeRight.size() - 1; i >= rowBreakRightDown; i--)
                            {
                                track.pointsEdgeRight[i].y = (int)(k * track.pointsEdgeRight[i].x + b);
                            }
                        }
                    }
                }

                track.pointsEdgeLeft.resize(track.crossroadSlant.x);  //切除顶部错误路径
                track.pointsEdgeRight.resize(track.crossroadSlant.x); //切除顶部错误路径
                res = true;                                           //返回十字识别结果
            }
        }
        else if (searchStraightCrossroad(track.pointsEdgeLeft, track.pointsEdgeRight)) //直入十字搜索
            crossroadType = CrossroadType::CrossroadStraight;

        if (crossroadType == CrossroadType::CrossroadStraight) //直入十字
        {
            rowBreakLeftUp = searchBreakLeftUp(track.pointsEdgeLeft);        //搜索十字赛道突变行（左上）
            rowBreakLeftDown = searchBreakLeftDown(track.pointsEdgeLeft);    //搜索十字赛道突变行（左下）
            rowBreakRightUp = searchBreakRightUp(track.pointsEdgeRight);     //搜索十字赛道突变行（右上）
            rowBreakRightDown = searchBreakRightDown(track.pointsEdgeRight); //搜索十字赛道突变行（右下
            crossroadType = CrossroadType::CrossroadStraight;
            _pointLU = track.pointsEdgeLeft[rowBreakLeftUp];
            _pointLD = track.pointsEdgeLeft[rowBreakLeftDown];
            _pointRU = track.pointsEdgeRight[rowBreakRightUp];
            _pointRD = track.pointsEdgeRight[rowBreakRightDown];

            if (rowBreakLeftUp > rowBreakLeftDown) //左边缘重绘赛道线
            {
                k = (float)(track.pointsEdgeLeft[rowBreakLeftUp].y - track.pointsEdgeLeft[rowBreakLeftDown].y) / (float)(track.pointsEdgeLeft[rowBreakLeftUp].x - track.pointsEdgeLeft[rowBreakLeftDown].x);
                b = track.pointsEdgeLeft[rowBreakLeftUp].y - k * track.pointsEdgeLeft[rowBreakLeftUp].x;
                for (int i = rowBreakLeftDown; i < rowBreakLeftUp; i++)
                {
                    track.pointsEdgeLeft[i].y = max((int)(k * track.pointsEdgeLeft[i].x + b), 0);
                }
            }
            else if (rowBreakLeftUp <= rowBreakLeftDown - 5) //仅找到左上补线点
            {
                int repairUp = (rowBreakLeftUp + rowBreakLeftDown) / 2;
                k = (float)(track.pointsEdgeLeft[repairUp].y - track.pointsEdgeLeft[rowBreakLeftUp].y) / (float)(track.pointsEdgeLeft[repairUp].x - track.pointsEdgeLeft[rowBreakLeftUp].x);
                b = track.pointsEdgeLeft[repairUp].y - k * track.pointsEdgeLeft[repairUp].x;

                uint16_t middleBreak = (rowBreakLeftUp + rowBreakLeftDown) / 2;
                if (track.pointsEdgeLeft[middleBreak].x < ROWSIMAGE / 2) //往下补线
                {
                    for (int i = 0; i < rowBreakLeftUp; i++)
                    {
                        track.pointsEdgeLeft[i].y = max((int)(k * track.pointsEdgeLeft[i].x + b), 0);
                    }
                }
                else //往上补线
                {
                    for (int i = rowBreakLeftDown; i < track.pointsEdgeLeft.size(); i++)
                    {
                        track.pointsEdgeLeft[i].y = max((int)(k * track.pointsEdgeLeft[i].x + b), 0);
                    }
                }
            }

            if (rowBreakRightUp > rowBreakRightDown) //右边缘重绘赛道线
            {
                k = (float)(track.pointsEdgeRight[rowBreakRightUp].y - track.pointsEdgeRight[rowBreakRightDown].y) / (float)(track.pointsEdgeRight[rowBreakRightUp].x - track.pointsEdgeRight[rowBreakRightDown].x);
                b = track.pointsEdgeRight[rowBreakRightUp].y - k * track.pointsEdgeRight[rowBreakRightUp].x;
                for (int i = rowBreakRightDown; i < rowBreakRightUp; i++)
                {
                    track.pointsEdgeRight[i].y = min((int)(k * track.pointsEdgeRight[i].x + b), COLSIMAGE - 1);
                }
            }
            else if (rowBreakRightUp <= rowBreakRightDown - 5) //仅找到右上补线点
            {
                int repairUp = (rowBreakRightUp + rowBreakRightDown) / 2;
                k = (float)(track.pointsEdgeRight[repairUp].y - track.pointsEdgeRight[rowBreakRightUp].y) / (float)(track.pointsEdgeRight[repairUp].x - track.pointsEdgeRight[rowBreakRightUp].x);
                b = track.pointsEdgeRight[repairUp].y - k * track.pointsEdgeRight[repairUp].x;

                uint16_t middleBreak = (rowBreakRightUp + rowBreakRightDown) / 2;
                if (track.pointsEdgeRight[middleBreak].x < ROWSIMAGE / 2) //往下补线
                {
                    for (int i = 0; i < rowBreakRightUp; i++)
                    {
                        track.pointsEdgeRight[i].y = min((int)(k * track.pointsEdgeRight[i].x + b), COLSIMAGE - 1);
                    }
                }
                else //往上补线
                {
                    for (int i = rowBreakRightDown; i < track.pointsEdgeRight.size(); i++)
                    {
                        track.pointsEdgeRight[i].y = min((int)(k * track.pointsEdgeRight[i].x + b), COLSIMAGE - 1);
                    }
                }
            }
            res = true; //返回十字识别结果
        }

        return res;
    }

    /**
     * @brief 绘制十字道路识别结果
     *
     * @param Image 需要叠加显示的图像/RGB
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
        for (int i = 0; i < track.spurroad.size(); i++)
        {
            circle(Image, Point(track.spurroad[i].y, track.spurroad[i].x), 6,
                   Scalar(0, 0, 255), -1); //红色点
        }

        //斜入十字绘制补线起止点
        if (crossroadType == CrossroadType::CrossroadRight) //右入十字
        {
            circle(Image, Point(_pointLU.y, _pointLU.x), 5, Scalar(226, 43, 138), -1); //上补线点：紫色
            circle(Image, Point(_pointLD.y, _pointLD.x), 5, Scalar(255, 0, 255), -1);  //下补线点：粉色
            if (_pointRU.x > 0)
                circle(Image, Point(_pointRU.y, _pointRU.x), 5, Scalar(226, 43, 138), -1); //上补线点：紫色
            if (_pointRD.x > 0)
                circle(Image, Point(_pointRD.y, _pointRD.x), 5, Scalar(255, 0, 255), -1); //下补线点：粉色

            putText(Image, "Right | " + to_string(track.rowsCrossroadStraight),
                    Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        }
        else if (crossroadType == CrossroadType::CrossroadLeft) //左入十字
        {
            circle(Image, Point(_pointRU.y, _pointRU.x), 5, Scalar(226, 43, 138), -1); //上补线点：紫色
            circle(Image, Point(_pointRD.y, _pointRD.x), 5, Scalar(255, 0, 255), -1);  //下补线点：粉色
            if (_pointLU.x > 0)
                circle(Image, Point(_pointLU.y, _pointLU.x), 5, Scalar(226, 43, 138), -1); //上补线点：紫色
            if (_pointLD.x > 0)
                circle(Image, Point(_pointLD.y, _pointLD.x), 5, Scalar(255, 0, 255), -1); //下补线点：粉色

            putText(Image, "Left | " + to_string(track.rowsCrossroadStraight),
                    Point(COLSIMAGE / 2 - 15, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        }
        else if (crossroadType == CrossroadType::CrossroadStraight) //直入十字
        {
            circle(Image, Point(_pointLU.y, _pointLU.x), 5, Scalar(226, 43, 138), -1); //上补线点：紫色
            circle(Image, Point(_pointLD.y, _pointLD.x), 5, Scalar(255, 0, 255), -1);  //下补线点：粉色
            circle(Image, Point(_pointRU.y, _pointRU.x), 5, Scalar(226, 43, 138), -1); //上补线点：紫色
            circle(Image, Point(_pointRD.y, _pointRD.x), 5, Scalar(255, 0, 255), -1);  //下补线点：粉色
            putText(Image, "Straight | " + to_string(track.rowsCrossroadStraight),
                    Point(COLSIMAGE / 2 - 20, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);
        }
        else
            putText(Image, to_string(track.rowsCrossroadStraight),
                    Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

        putText(Image, "Cross", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
    }

private:
    bool repairLeftEnable = false;  //左边缘补线使能标志
    bool repairRightEnable = false; //右边缘补线使能标志

    uint16_t _index = 0;
    POINT _pointLU;
    POINT _pointLD;
    POINT _pointRU;
    POINT _pointRD;
    string _text;

    /**
     * @brief 十字道路类型
     *
     */
    enum CrossroadType
    {
        None = 0,
        CrossroadLeft,     //左斜入十字
        CrossroadRight,    //右斜入十字
        CrossroadStraight, //直入十字
    };

    CrossroadType crossroadType = CrossroadType::None; //十字道路类型
};