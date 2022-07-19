#pragma once
/**
 * @file busyarea_detection.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 施工区检测与路径规划
 * @version 0.1
 * @date 2022-03-30
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

class BusyareaDetection
{
public:
    bool slowDown = false; //减速使能

    /**
     * @brief 施工区检测初始化
     *
     */
    void reset(void)
    {
        busyareaStep = BusyareaStep::BusyareaNone;
        counterSession = 0;         //图像场次计数器
        counterRec = 0;             //施工区标志检测计数器
        lastPointsEdgeLeft.clear(); //记录上一场边缘点集（丢失边）
        lastPointsEdgeRight.clear();
    }

    /**
     * @brief 施工区检测与路径规划
     *
     * @param track 赛道识别结果
     * @param detection AI检测结果
     */
    bool busyareaDetection(TrackRecognition &track, vector<PredictResult> predict)
    {
        slowDown = false;
        _pointNearCone = POINT(0, 0);
        pointEdgeDet.clear();

        switch (busyareaStep)
        {
        case BusyareaStep::BusyareaNone: //[01] 施工区标志检测
            for (int i = 0; i < predict.size(); i++)
            {
                if (predict[i].label == LABEL_BUSYAREA) //施工区标志检测
                {
                    counterRec++;
                    break;
                }
            }
            if (counterRec)
            {
                counterSession++;
                if (counterRec > 3 && counterSession < 8)
                {
                    busyareaStep = BusyareaStep::BusyareaEnable; //施工区使能
                    counterRec = 0;
                    counterSession = 0;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            break;

        case BusyareaStep::BusyareaEnable: //[02] 施工区使能
        {
            vector<POINT> pointsCone = searchCones(predict);
            _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointsCone); //搜索右下锥桶
            if (_pointNearCone.x > ROWSIMAGE * 0.1)                               //当车辆开始靠近右边锥桶：准备入库
            {
                counterRec++;
                if (counterRec >= 2)
                {
                    busyareaStep = BusyareaStep::BusyareaEnter; //进站使能
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            else if (_pointNearCone.x > 0.1 && _pointNearCone.x < ROWSIMAGE * 0.4)
            {
                slowDown = true; //进展减速
            }
            break;
        }

        case BusyareaStep::BusyareaEnter: //[03] 进站使能
        {
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2) //第一阶段：当赛道边缘存在时
            {
                vector<POINT> pointsCone = searchCones(predict);                      //计算锥桶的坐标
                _pointNearCone = searchNearestCone(track.pointsEdgeLeft, pointsCone); //搜索右下锥桶
                if (_pointNearCone.x > 0)                                             //坐标有效
                {
                    POINT startPoint = POINT((_pointNearCone.x + ROWSIMAGE) / 2, (_pointNearCone.y + COLSIMAGE) / 2); //线起点：右
                    double k = 0, b = 0;
                    k = (float)(_pointNearCone.y - startPoint.y) / (float)(_pointNearCone.x - startPoint.x);
                    b = _pointNearCone.y - k * _pointNearCone.x;

                    if (b < 0)
                        b = 0;
                    else if (b >= COLSIMAGE)
                        b = COLSIMAGE - 1;
                    POINT endPoint = POINT(0, b);    //补线终点：左
                    POINT midPoint = _pointNearCone; //补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = Bezier(0.02, input);
                    track.pointsEdgeRight = repair;
                    track.pointsEdgeLeft.clear();

                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeLeft.push_back(POINT(repair[i].x, 0));
                    }
                }
            }
            else //第二阶段：检查右下锥桶坐标满足巡航条件
            {
                vector<POINT> pointsCone = searchCones(predict);       //计算锥桶的坐标
                POINT coneRightDown = searchRightDownCone(pointsCone); //右下方锥桶
                _pointNearCone = coneRightDown;
                counterSession++;
                if ((coneRightDown.x > ROWSIMAGE / 3 && coneRightDown.y > COLSIMAGE - 80) || counterSession > 20)
                {
                    counterRec++;
                    if (counterRec >= 2)
                    {
                        busyareaStep = BusyareaStep::BusyareaCruise; //巡航使能
                        counterRec = 0;
                        counterSession = 0;
                    }
                }
                if (coneRightDown.x > 0) //进站补线
                {
                    POINT startPoint = POINT((coneRightDown.x + ROWSIMAGE) / 2, (coneRightDown.y + COLSIMAGE) / 2); //线起点：右
                    double k = 0, b = 0;
                    k = (float)(coneRightDown.y - startPoint.y) / (float)(coneRightDown.x - startPoint.x);
                    b = coneRightDown.y - k * coneRightDown.x;

                    if (b < 0)
                        b = 0;
                    else if (b >= COLSIMAGE)
                        b = COLSIMAGE - 1;
                    POINT endPoint = POINT(0, b);   //补线终点：左
                    POINT midPoint = coneRightDown; //补线中点
                    vector<POINT> input = {startPoint, midPoint, endPoint};
                    vector<POINT> repair = Bezier(0.02, input);
                    track.pointsEdgeRight = repair;
                    track.pointsEdgeLeft.clear();

                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeLeft.push_back(POINT(repair[i].x, 0));
                    }
                }
            }

            break;
        }

        case BusyareaStep::BusyareaCruise: //[04] 巡航使能
        {
            vector<POINT> pointsCone = searchCones(predict);      //计算锥桶的坐标
            vector<POINT> conesLeft = searchLeftCone(pointsCone); //搜索左方锥桶

            // if (pointsCone.size() < 2 && track.pointsEdgeLeft.size() > ROWSIMAGE / 2 && track.pointsEdgeRight.size() > ROWSIMAGE / 2)
            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 6 && track.pointsEdgeRight.size() > ROWSIMAGE / 6)
            {
                slowDown = true; //出站减速
                counterRec++;
                if (counterRec >= 2)
                {
                    busyareaStep = BusyareaStep::BusyareaExit; //出站使能
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            else
                counterRec = 0;

            if (conesLeft.size() >= 2)
            {
                int indexMin = 0;
                int indexMax = 0;
                for (int i = 0; i < conesLeft.size(); i++)
                {
                    if (conesLeft[i].x > conesLeft[indexMax].x)
                        indexMax = i;
                    if (conesLeft[i].x < conesLeft[indexMin].x)
                        indexMin = i;
                }

                if (indexMin != indexMax) //开始补线
                {
                    double k = 0, b = 0;
                    k = (float)(conesLeft[indexMax].y - conesLeft[indexMin].y) / (float)(conesLeft[indexMax].x - conesLeft[indexMin].x);
                    b = conesLeft[indexMax].y - k * conesLeft[indexMax].x;

                    if (k != 0 && b != 0)
                    {
                        POINT startPoint = POINT(-b / k, 0);                                                          //补线起点：左
                        POINT endPoint = POINT(0, b);                                                                 //补线终点：右
                        POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); //补线中点
                        vector<POINT> input = {startPoint, midPoint, endPoint};
                        vector<POINT> repair = Bezier(0.02, input);

                        track.pointsEdgeRight = predictEdgeRight(repair); //俯视域预测右边缘

                        if (repair.size() > 10) //左边缘切行，提升右拐能力
                        {
                            int index = repair.size() * 0.3;
                            track.pointsEdgeLeft.clear();
                            for (int i = index; i < repair.size(); i++)
                            {
                                track.pointsEdgeLeft.push_back(repair[i]);
                            }
                        }
                        else
                            track.pointsEdgeLeft = repair;

                        lastPointsEdgeLeft = track.pointsEdgeLeft;
                    }
                }
            }
            else if (pointsCone.size() > 3)
            {
                track.pointsEdgeLeft = lastPointsEdgeLeft;
                track.pointsEdgeRight = predictEdgeRight(track.pointsEdgeLeft); //俯视域预测右边缘
            }

            //检查进站情况
            counterSession++;
            POINT coneRightDown = searchRightDownCone(pointsCone);                               //右下方锥桶
            if ((coneRightDown.x < ROWSIMAGE / 2 && counterSession > 12) || counterSession > 30) //右下方锥桶检测完毕
            {
                busyareaStep = BusyareaStep::BusyareaExit; //出站使能
                counterRec = 0;
                counterSession = 0;
            }

            break;
        }
        // case BusyareaStep::BusyareaExit: //[05] 出站使能
        // {
        //     slowDown = true;                                                                                 //出站减速
        //     if (track.pointsEdgeLeft.size() > ROWSIMAGE / 6 && track.pointsEdgeRight.size() > ROWSIMAGE / 6) //右边缘切行，提升出站左拐能力
        //     {
        //         for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        //         {
        //             if (track.pointsEdgeRight[i].y > COLSIMAGE * 0.6)
        //                 track.pointsEdgeRight[i].y = COLSIMAGE * 0.6;
        //         }
        //     }
        //     else
        //     {
        //         track.pointsEdgeLeft.clear();
        //         track.pointsEdgeRight.clear();
        //     }

        //     counterRec++;
        //     if (counterRec > 10)
        //     {
        //         busyareaStep = BusyareaStep::BusyareaNone; //施工区结束
        //         counterRec = 0;
        //         counterSession = 0;
        //     }
        //     break;
        // }
        case BusyareaStep::BusyareaExit: //[05] 出站使能
        {
            vector<POINT> pointsCone = searchCones(predict);  //计算锥桶的坐标
            POINT coneLeftUp = searchRightUpCone(pointsCone); //搜索左上方的锥桶用于补线

            if (track.pointsEdgeLeft.size() > ROWSIMAGE / 4 && track.pointsEdgeRight.size() > ROWSIMAGE / 4 && pointsCone.size() < 3)
            {
                busyareaStep = BusyareaStep::BusyareaNone; //出站结束
                counterRec = 0;
                counterSession = 0;
            }
            else
            {
                if (coneLeftUp.x > 0)
                {
                    POINT p1 = POINT(ROWSIMAGE - 10, coneLeftUp.y / 2);
                    POINT p2 = POINT((coneLeftUp.x + ROWSIMAGE) / 2, coneLeftUp.y / 2);
                    POINT p3 = coneLeftUp;
                    POINT p4 = POINT(coneLeftUp.x / 2, (coneLeftUp.y + COLSIMAGE) / 2);
                    vector<POINT> input = {p1, p2, p3, p4};
                    vector<POINT> repair = Bezier(0.02, input);

                    track.pointsEdgeLeft = repair;
                    lastPointsEdgeLeft = repair;
                    track.pointsEdgeRight.clear();
                    for (int i = 0; i < repair.size(); i++)
                    {
                        track.pointsEdgeRight.push_back(POINT(repair[i].x, COLSIMAGE - 1));
                    }
                    lastPointsEdgeRight = track.pointsEdgeRight;
                }
                else
                {
                    track.pointsEdgeLeft = lastPointsEdgeLeft;
                    track.pointsEdgeRight = lastPointsEdgeRight;
                }
            }

            break;
        }
        }

        if (busyareaStep == BusyareaStep::BusyareaNone) //返回施工区控制模式标志
            return false;
        else
            return true;
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void
    drawImage(TrackRecognition track, Mat &imageBusyarea)
    {
        for (int i = 0; i < pointEdgeDet.size(); i++)
        {
            circle(imageBusyarea, Point(pointEdgeDet[i].y, pointEdgeDet[i].x), 2, Scalar(92, 92, 205), -1); //锥桶坐标：红色
        }

        // 赛道边缘
        for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(imageBusyarea, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); //绿色点
        }
        for (int i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(imageBusyarea, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); //黄色点
        }

        //显示施工区状态
        string state = "None";
        switch (busyareaStep)
        {
        case BusyareaEnable:
            state = "Enable";
            break;
        case BusyareaEnter:
            state = "Enter";
            break;
        case BusyareaCruise:
            state = "Cruise";
            break;
        case BusyareaExit:
            state = "Exit";
            break;
        }
        putText(imageBusyarea, state, Point(COLSIMAGE / 2 - 10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);

        if (_pointNearCone.x > 0)
            circle(imageBusyarea, Point(_pointNearCone.y, _pointNearCone.x), 3, Scalar(200, 200, 200), -1);
    }

private:
    POINT _pointNearCone;
    vector<POINT> pointEdgeDet;       // AI元素检测边缘点集
    vector<POINT> lastPointsEdgeLeft; //记录上一场边缘点集（丢失边）
    vector<POINT> lastPointsEdgeRight;

    enum BusyareaStep
    {
        BusyareaNone = 0, //未触发
        BusyareaEnable,   //施工区操作使能（标志识别成功）
        BusyareaEnter,    //施工区进站
        BusyareaCruise,   //施工区巡航
        BusyareaExit      //施工区出站
    };

    BusyareaStep busyareaStep = BusyareaStep::BusyareaNone;
    uint16_t counterSession = 0; //图像场次计数器
    uint16_t counterRec = 0;     //施工区标志检测计数器

    /**
     * @brief 从AI检测结果中检索锥桶坐标集合
     *
     * @param predict AI检测结果
     * @return vector<POINT>
     */
    vector<POINT> searchCones(vector<PredictResult> predict)
    {
        vector<POINT> cones;
        for (int i = 0; i < predict.size(); i++)
        {
            if (predict[i].label == LABEL_CONE) //锥桶检测
            {
                cones.push_back(POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2));
            }
        }

        pointEdgeDet = cones;
        return cones;
    }

    /**
     * @brief 搜索距离赛道左边缘最近的锥桶坐标
     *
     * @param pointsEdgeLeft 赛道边缘点集
     * @param predict AI检测结果
     * @return POINT
     */
    POINT searchNearestCone(vector<POINT> pointsEdgeLeft, vector<POINT> pointsCone)
    {
        POINT point(0, 0);
        double disMin = 50; //右边缘锥桶离赛道左边缘最小距离

        if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
            return point;

        POINT a = pointsEdgeLeft[pointsEdgeLeft.size() / 4];
        POINT b = pointsEdgeLeft[pointsEdgeLeft.size() / 2];

        for (int i = 0; i < pointsCone.size(); i++)
        {
            double dis = distanceForPoint2Line(a, b, pointsCone[i]);
            if (dis < disMin && pointsCone[i].x > point.x)
            {
                point = pointsCone[i];
            }
        }

        return point;
    }

    /**
     * @brief 搜索右下方的锥桶坐标
     *
     * @param pointsCone
     * @return POINT
     */
    POINT searchRightDownCone(vector<POINT> pointsCone)
    {
        POINT point(0, 0);

        if (pointsCone.size() <= 0)
            return point;

        int index = 0;
        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y > COLSIMAGE / 2 && pointsCone[i].x > point.x)
            {
                point = pointsCone[i];
            }
        }

        return point;
    }

    /**
     * @brief 搜索左方的锥桶坐标
     *
     * @param pointsCone
     * @return vector<POINT>
     */
    vector<POINT> searchLeftCone(vector<POINT> pointsCone)
    {
        vector<POINT> points;

        if (pointsCone.size() <= 0)
            return points;

        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y < COLSIMAGE / 2)
            {
                points.push_back(pointsCone[i]);
            }
        }

        return points;
    }

    /**
     * @brief 搜索右上方的锥桶坐标
     *
     * @param pointsCone
     * @return vector<POINT>
     */
    POINT searchRightUpCone(vector<POINT> pointsCone)
    {
        POINT point(0, 0);

        if (pointsCone.size() <= 0)
            return point;

        for (int i = 0; i < pointsCone.size(); i++)
        {
            if (pointsCone[i].y > point.y && pointsCone[i].x < ROWSIMAGE * 0.8)
            {
                point = pointsCone[i];
            }
        }

        return point;
    }

    vector<POINT> predictEdgeRight(vector<POINT> pointsEdgeLeft)
    {
        int offset = 200; //右边缘平移尺度
        vector<POINT> pointsEdgeRight;
        POINT startPoint(0, 0);
        POINT endPoint(0, 0);

        if (pointsEdgeLeft.size() < 3)
            return pointsEdgeRight;

        // Start
        Point2d startIpm = ipm.homography(Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); //透视变换
        Point2d prefictRight;
        if (startIpm.x + offset >= COLSIMAGEIPM) //平移边缘
            return pointsEdgeRight;
        else
            prefictRight = Point2d(startIpm.x + offset, startIpm.y);

        Point2d startIipm = ipm.homographyInv(prefictRight); //反透视变换
        startPoint = POINT(startIipm.y, startIipm.x);

        // End
        Point2d endIpm = ipm.homography(Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y, pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); //透视变换
        prefictRight = Point2d(endIpm.x + offset, endIpm.y);
        Point2d endtIipm = ipm.homographyInv(prefictRight); //反透视变换
        endPoint = POINT(endtIipm.y, endtIipm.x);

        //补线
        POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); //补线中点
        vector<POINT> input = {startPoint, midPoint, endPoint};
        vector<POINT> repair = Bezier(0.02, input);

        for (int i = 0; i < repair.size(); i++)
        {
            if (repair[i].x >= ROWSIMAGE)
                repair[i].x = ROWSIMAGE - 1;

            else if (repair[i].x < 0)
                repair[i].x = 0;

            else if (repair[i].y >= COLSIMAGE)
                repair[i].y = COLSIMAGE - 1;
            else if (repair[i].y < 0)
                repair[i].y = 0;

            pointsEdgeRight.push_back(repair[i]);
        }

        return pointsEdgeRight;
    }
};