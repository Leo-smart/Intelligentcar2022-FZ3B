#pragma once
/**
 * @file garage_recognition.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 车库识别
 * @version 0.1
 * @date 2022-03-14
 * @copyright Copyright (c) 2022
 * @note 车库识别步骤:
 *                  [01] 车库标志识别：1）斑马线
 *                  [02] 赛道左边缘优化,入库点搜索
 *                  [03] 入库路径优化
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "track_recognition.cpp"
#include "../../include/predictor.hpp"

using namespace cv;
using namespace std;

class GarageRecognition
{
public:
  bool slowDown = false;      //减速使能
  bool entryEnable = false;   //入库使能
  float disGarageEntry = 0.7; //车库入库距离(斑马线Image占比)

  /**
   * @brief 出库步骤
   *
   */
  enum GarageStep
  {
    GarageExiting = 0,      //出库中
    GarageEntryRecognition, //入库标志识别
    GarageEntryingA,        //入库中：A阶段
    GarageEntryingB,        //入库中：B阶段
    GarageEntryFinish       //入库完成
  };

  GarageStep garageStep = GarageStep::GarageExiting;

  /**
   * @brief 起点检测
   *
   * @param predict
   * @return true
   * @return false
   */
  bool startingCheck(vector<PredictResult> predict)
  {
    if (startingFilt)
    {
      for (int i = 0; i < predict.size(); i++)
      {
        if (predict[i].label == LABEL_CROSSWALK) //标志检测
        {
          counterCrosswalk++;
          break;
        }
      }

      if (counterCrosswalk)
      {
        counterSessionTwo++;

        if (counterCrosswalk > 5 && counterSessionTwo < 12)
        {
          counterSessionTwo = 0;
          counterCrosswalk = 0;
          return true;
        }
        else if (counterSessionTwo > 12)
        {
          counterSessionTwo = 0;
          counterCrosswalk = 0;
        }
      }
    }
    else if (garageStep != GarageStep::GarageExiting) //出库屏蔽
    {
      counterSessionTwo++;
      if (counterSessionTwo > 50)
      {
        counterSessionTwo = 0;
        counterCrosswalk = 0;
        startingFilt = true;
      }
    }

    return false;
  }

  /**
   * @brief 车库识别与路径规划（入库与出库）
   *
   * @param track
   */
  bool garageRecognition(TrackRecognition &track, vector<PredictResult> predict)
  {
    if (garageStep == GarageStep::GarageExiting) //出库阶段
    {
      garageExitRecognition(track);
    }
    else if (entryEnable) //入库阶段：入库使能
    {
      garageEntryRecognition(track, predict);
    }

    if (garageStep == GarageExiting || garageStep == GarageEntryingA || garageStep == GarageEntryingB || garageStep == GarageEntryFinish)
      return true;
    else
      return false;
  }

  /**
   * @brief 入库识别与路径规划
   *
   * @param track 基础赛道识别结果
   */
  void garageEntryRecognition(TrackRecognition &track, vector<PredictResult> predict)
  {
    _pointRU = POINT(0, 0);
    _pointLU = POINT(0, 0);
    _pointRD = POINT(0, 0);
    _Index = "-";
    slowDown = false;

    POINT crosswalk = searchCrosswalkSign(predict);
    if (crosswalk.x > 0)
      counterRec++;
    else
      counterRec = 0;

    if (counterRec > 3 && garageStep == GarageEntryRecognition) //通过AI标志提前识别车库|避免误识别环岛
    {
      garageStep = GarageEntryingA;
    }

    if (counterRec > 3 && crosswalk.x > ROWSIMAGE / 2) // AI识别到斑马线：减速，精确识别入库图像
    {
      slowDown = true;
    }

    if (track.garageEnable.x || garageStep == GarageStep::GarageEntryingA) //车库识别标志
    {
      counterEnterA++;
      if (counterEnterA > 5)
        garageStep = GarageStep::GarageEntryingA;

      _Index = "-------";
      //[02] 赛道右边缘优化
      uint16_t rowBreakLeftUp = searchBreakLeftUp(track.pointsEdgeLeft); //搜索车库入库点（左上库点）
      _pointLU = track.pointsEdgeLeft[rowBreakLeftUp];

      if (track.pointsEdgeRight.size() > rowBreakLeftUp)
        _pointRU = track.pointsEdgeRight[rowBreakLeftUp];
      else
        return;

      if (rowBreakLeftUp > track.garageEnable.y && rowBreakLeftUp < track.pointsEdgeRight.size() - 1) //上库点正确性校验
      {
        _Index = "1";
        uint16_t rowBreaRightDown = searchBreakRightDown(track.pointsEdgeRight); //右边缘突变点（右下）
        _pointRD = track.pointsEdgeRight[rowBreaRightDown];

        if (rowBreaRightDown < track.garageEnable.y && track.pointsEdgeRight[rowBreaRightDown].x > ROWSIMAGE * 0.75) //远处的斑马线
        {
          _Index = "2";
          if (track.pointsEdgeRight[rowBreakLeftUp].x == track.pointsEdgeRight[rowBreaRightDown].x)
            return;

          if (track.pointsEdgeRight[track.garageEnable.y].x > ROWSIMAGE * disGarageEntry) //防止提前入库
          {
            double k, b;
            k = (float)(track.pointsEdgeRight[rowBreakLeftUp].y - track.pointsEdgeRight[rowBreaRightDown].y) / (float)(track.pointsEdgeRight[rowBreakLeftUp].x - track.pointsEdgeRight[rowBreaRightDown].x);
            b = track.pointsEdgeRight[rowBreakLeftUp].y - k * track.pointsEdgeRight[rowBreakLeftUp].x;

            int rowRepairDown = rowBreakLeftUp - (rowBreakLeftUp - track.garageEnable.y) * 2;
            if (rowRepairDown > rowBreaRightDown) //远距离斑马线补线
            {
              _Index = "3";

              track.pointsEdgeRight[rowRepairDown].y = (int)(k * track.pointsEdgeRight[rowRepairDown].x + b);

              POINT startPoint = track.pointsEdgeRight[rowRepairDown]; //入库补线起点：右
              POINT endPoint = track.pointsEdgeLeft[rowBreakLeftUp];   //入库补线终点：左

              int midX = (startPoint.x + endPoint.x) * 0.25;
              int midY = (startPoint.y + endPoint.y) * 0.4;
              if (midX < endPoint.x) //数据阈值
                midX = endPoint.x;
              else if (midX > startPoint.x)
                midX = startPoint.x;
              if (midY > startPoint.y)
                midY = startPoint.y;
              else if (midY < endPoint.y)
                midY = endPoint.y;

              POINT midPoint = POINT(midX, midY); //入库补线中点

              vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
              vector<POINT> modifyEdge = Bezier(0.02, repairPoints); //三阶贝塞尔曲线拟合
              track.pointsEdgeRight.resize(rowRepairDown);           //删除无效点
              track.pointsEdgeLeft.resize(rowBreakLeftUp);           //删除无效点
              for (int i = 0; i < modifyEdge.size(); i++)
              {
                track.pointsEdgeRight.push_back(modifyEdge[i]);
              }
            }
            else //靠近斑马线
            {
              _Index = "4";

              if (track.pointsEdgeLeft[rowBreakLeftUp].y > COLSIMAGE / 2 - 20)
              {
                track.pointsEdgeLeft = lastPointsEdgeLeft;
                track.pointsEdgeRight = lastPointsEdgeRight;
              }
              else
              {
                POINT startPoint = POINT(ROWSIMAGE - 10, COLSIMAGE / 2);                                      //入库补线起点:固定左下角
                POINT endPoint = track.pointsEdgeLeft[rowBreakLeftUp];                                        //入库补线终点
                POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.3, (startPoint.y + endPoint.y) * 0.4); //入库补线中点
                vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                vector<POINT> modifyEdgeLeft = Bezier(0.02, repairPoints); //三阶贝塞尔曲线拟合
                track.pointsEdgeRight = modifyEdgeLeft;                    //左边缘重新赋值
                track.pointsEdgeLeft = modifyEdgeLeft;
                for (int i = 0; i < modifyEdgeLeft.size(); i++) //矫正右边缘坐标：最值
                {
                  track.pointsEdgeLeft[i].y = 0;
                }
                if (modifyEdgeLeft.size() > 3) //防止有效行计算切单边
                {
                  track.pointsEdgeLeft[modifyEdgeLeft.size() - 1].y = 3;
                  track.pointsEdgeLeft[modifyEdgeLeft.size() - 2].y = 3;
                }
              }
            }
          }
        }
        else //车头覆盖斑马线
        {
          _Index = "5";
          if (track.pointsEdgeRight[rowBreakLeftUp].y - track.pointsEdgeLeft[rowBreakLeftUp].y < COLSIMAGE / 4) //入库状态切换
          {
            garageStep = GarageStep::GarageEntryingB;
          }
          else
          {
            if (track.pointsEdgeRight[track.garageEnable.y].x > ROWSIMAGE * disGarageEntry) //防止提前入库
            {
              if (track.pointsEdgeLeft[rowBreakLeftUp].y > COLSIMAGE / 2 - 20)
              {
                track.pointsEdgeLeft = lastPointsEdgeLeft;
                track.pointsEdgeRight = lastPointsEdgeRight;
              }
              else
              {
                POINT startPoint = POINT(ROWSIMAGE - 10, COLSIMAGE / 2);                                      //入库补线起点:固定左下角
                POINT endPoint = track.pointsEdgeLeft[rowBreakLeftUp];                                        //入库补线终点
                POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5); //入库补线中点
                vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
                vector<POINT> modifyEdgeLeft = Bezier(0.02, repairPoints); //三阶贝塞尔曲线拟合
                track.pointsEdgeRight = modifyEdgeLeft;                    //左边缘重新赋值
                track.pointsEdgeLeft = modifyEdgeLeft;
                for (int i = 0; i < modifyEdgeLeft.size(); i++) //矫正右边缘坐标：最值
                {
                  track.pointsEdgeLeft[i].y = 0;
                }
                if (modifyEdgeLeft.size() > 3) //防止有效行计算切单边
                {
                  track.pointsEdgeLeft[modifyEdgeLeft.size() - 1].y = 3;
                  track.pointsEdgeLeft[modifyEdgeLeft.size() - 2].y = 3;
                }
              }
            }
          }
        }

        lastPointsEdgeLeft = track.pointsEdgeLeft;
        lastPointsEdgeRight = track.pointsEdgeRight;
      }
    }

    if (garageStep == GarageStep::GarageEntryingA)
    {
      if (track.pointsEdgeLeft.size() < 5 || track.pointsEdgeRight.size() < 5)
      {
        counterEntryOut++;
        if (counterEntryOut > 5)
          garageStep = GarageStep::GarageEntryFinish;
      }
      else if (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x > ROWSIMAGE / 3 && track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x > ROWSIMAGE / 3)
      {
        counterEntryOut++;
        if (counterEntryOut > 5)
          garageStep = GarageStep::GarageEntryFinish;
      }
    }

    if (garageStep == GarageStep::GarageEntryingB)
    {
      if (track.pointsEdgeLeft.size() < 5 || track.pointsEdgeRight.size() < 5)
      {
        counterEntryOut++;
        if (counterEntryOut > 5)
          garageStep = GarageStep::GarageEntryFinish;
      }
      else if (track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x > ROWSIMAGE / 3 && track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x > ROWSIMAGE / 3)
      {
        counterEntryOut++;
        if (counterEntryOut > 5)
          garageStep = GarageStep::GarageEntryFinish;
      }
      else if (track.spurroad.size() > 0) //还未完全驶入路口
      {
        POINT startPoint = track.spurroad[0];                                                         //入库补线起点:固定左下角
        POINT endPoint = POINT(startPoint.x / 2, 0);                                                  //入库补线终点
        POINT midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); //入库补线中点
        vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
        vector<POINT> modifyEdge = Bezier(0.02, repairPoints); //三阶贝塞尔曲线拟合

        track.pointsEdgeRight = modifyEdge;
        track.pointsEdgeLeft = modifyEdge;
        for (int i = 0; i < modifyEdge.size() - 2; i++)
        {
          track.pointsEdgeLeft[i].y = 0;
        }

        if (modifyEdge.size() > 3) //防止有效行计算切单边
        {
          track.pointsEdgeLeft[modifyEdge.size() - 1].y = 3;
          track.pointsEdgeLeft[modifyEdge.size() - 2].y = 3;
        }
      }
      else if (track.pointsEdgeLeft.size() > ROWSIMAGE * 0.7 && track.pointsEdgeRight.size() > ROWSIMAGE * 0.7)
      {
        track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() * 0.8);
        track.pointsEdgeRight.resize(track.pointsEdgeRight.size() * 0.8);
        counterEntryOut = 0;
      }
    }
  }

  /**
   * @brief 出库识别与路径规划
   *
   * @param track
   */
  void garageExitRecognition(TrackRecognition &track)
  {
    _pointRU = POINT(0, 0);
    _pointLU = POINT(0, 0);
    _pointRD = POINT(0, 0);
    _Index = "";

    if (garageStep == GarageStep::GarageExiting) //出库中
    {
      POINT startPoint;
      POINT endPoint;
      POINT midPoint;
      if (track.spurroad.size() < 3 || (track.stdevLeft < 100 && track.stdevRight < 100)) //离开斑马线
      {
        counterExitOut++;
        if (counterExitOut >= 5)
          garageStep = GarageStep::GarageEntryRecognition; //出库完成
      }
      else
      {
        counterExitOut = 0;
      }

      if (track.pointsEdgeLeft.size() <= 1 || track.pointsEdgeRight.size() <= 1) //图像异常
        return;

      uint16_t rowBreakLeft = searchBreakLeft(track.pointsEdgeLeft);    //左上拐点搜索
      uint16_t rowBreakRight = searchBreakRight(track.pointsEdgeRight); //右下补线点搜索

      if (track.pointsEdgeRight[rowBreakRight].x < ROWSIMAGE * 0.5) //避免出库提前转向优化
      {
        track.pointsEdgeRight.resize(rowBreakRight);
        if (rowBreakLeft > rowBreakRight)
          track.pointsEdgeLeft.resize(rowBreakLeft);

        return;
      }

      if (rowBreakLeft >= track.pointsEdgeRight.size())
      {
        rowBreakLeft = track.pointsEdgeRight.size() - 1;
      }

      startPoint = track.pointsEdgeRight[rowBreakRight]; //入库补线起点
      _pointLU = startPoint;
      _pointRD = track.pointsEdgeLeft[rowBreakLeft];

      //依赖岔路补线
      if (track.spurroad.size() > 2)
      {
        endPoint = searchBestSpurroad(track.spurroad); //入库补线终点

        if (startPoint.x > endPoint.x && startPoint.y > endPoint.y) //补线起点和终点正确性校验
        {
          // 斑马线右边部分补线
          midPoint = POINT((startPoint.x + endPoint.x) * 0.4, (startPoint.y + endPoint.y) * 0.5); //入库补线中点
          _pointRU = endPoint;
          vector<POINT> repairPoints = {startPoint, midPoint, endPoint};
          vector<POINT> modifyEdgeLeft = Bezier(0.04, repairPoints); //三阶贝塞尔曲线拟合
          track.pointsEdgeRight.resize(rowBreakRight);               //删除无效点
          for (int i = 0; i < modifyEdgeLeft.size(); i++)
          {
            track.pointsEdgeRight.push_back(modifyEdgeLeft[i]);
          }
          // 斑马线左边部分补线
          startPoint = endPoint;
          endPoint = track.pointsEdgeLeft[rowBreakLeft];
          midPoint = POINT((startPoint.x + endPoint.x) * 0.5, (startPoint.y + endPoint.y) * 0.5); //入库补线中点
          repairPoints = {startPoint, midPoint, endPoint};
          modifyEdgeLeft.resize(0);
          modifyEdgeLeft = Bezier(0.02, repairPoints); //三阶贝塞尔曲线拟合
          for (int i = 0; i < modifyEdgeLeft.size(); i++)
          {
            track.pointsEdgeRight.push_back(modifyEdgeLeft[i]);
          }
        }

        //左边缘错误点优化
        track.pointsEdgeLeft.resize(rowBreakLeft);
      }
    }
  }

  /**
   * @brief 车库识别结果图像绘制
   *
   * @param track 赛道识别结果
   * @param trackImage 输入叠加图像
   */
  void drawImage(TrackRecognition &track, Mat &trackImage)
  {
    // 赛道边缘
    for (int i = 0; i < track.pointsEdgeLeft.size(); i++)
    {
      circle(trackImage, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
             Scalar(0, 255, 0), -1); //绿色点
    }
    for (int i = 0; i < track.pointsEdgeRight.size(); i++)
    {
      circle(trackImage, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
             Scalar(0, 255, 255), -1); //黄色点
    }

    // 岔路点
    for (int i = 0; i < track.spurroad.size(); i++)
    {
      circle(trackImage, Point(track.spurroad[i].y, track.spurroad[i].x), 3,
             Scalar(0, 0, 255), -1); //红色点
    }

    circle(trackImage, Point(_pointRU.y, _pointRU.x), 5, Scalar(226, 43, 138), -1); //上库点：紫色点
    circle(trackImage, Point(_pointRD.y, _pointRD.x), 5, Scalar(226, 43, 138), -1); //上库点：紫色点
    circle(trackImage, Point(_pointLU.y, _pointLU.x), 5, Scalar(255, 0, 255), -1);  //下库点：粉色点

    putText(trackImage, _Index, Point(COLSIMAGE / 2 - 5, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

    putText(trackImage, "[1] Garage", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA); //显示赛道识别类型

    string step = "Exiting";
    switch (garageStep)
    {
    case GarageStep::GarageEntryRecognition:
      step = "EntryRec";
      break;
    case GarageStep::GarageEntryingA:
      step = "Entry|A";
      break;
    case GarageStep::GarageEntryingB:
      step = "Entry|B";
      break;
    case GarageStep::GarageEntryFinish:
      step = "Finish!";
      break;
    }
    putText(trackImage, step, Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA);

    putText(trackImage, "Garage", Point(COLSIMAGE / 2 - 5, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
  }

private:
  string _Index = "";
  POINT _pointRU;
  POINT _pointLU;
  POINT _pointRD;
  uint16_t counterEnterA = 0;        //入库状态A切换计数器
  uint16_t counterExitOut = 0;       //出库完成计数器
  uint16_t counterEntryOut = 0;      //入库完成计数器
  uint16_t counterSession = 0;       //图像场次计数器
  uint16_t counterRec = 0;           //标志检测计数器
  vector<POINT> lastPointsEdgeLeft;  //记录上一场边缘点集（丢失边）
  vector<POINT> lastPointsEdgeRight; //
  uint16_t counterCrosswalk = 0;     //起点检测计数器
  uint16_t counterSessionTwo = 0;
  bool startingFilt = false; //出库屏蔽标志

  /**
   * @brief 冒泡法求取集合中值
   *
   * @param vec 输入集合
   * @return int 中值
   */
  int getMiddleValue(vector<int> vec)
  {
    if (vec.size() < 1)
      return -1;
    if (vec.size() == 1)
      return vec[0];

    int len = vec.size();
    while (len > 0)
    {
      bool sort = true; //是否进行排序操作标志
      for (int i = 0; i < len - 1; ++i)
      {
        if (vec[i] > vec[i + 1])
        {
          swap(vec[i], vec[i + 1]);
          sort = false;
        }
      }
      if (sort) //排序完成
        break;

      --len;
    }

    return vec[(int)vec.size() / 2];
  }

  /**
   * @brief 搜索入库|赛道突变行（右上）
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
  uint16_t searchBreakLeftUp(vector<POINT> pointsEdgeLeft)
  {
    if (pointsEdgeLeft.size() < 5)
      return 0;

    uint16_t rowBreakLeftUp = pointsEdgeLeft.size() - 5;
    uint16_t counter = 0;
    uint16_t counterFilter = 0;
    for (int i = pointsEdgeLeft.size() - 5; i > 50; i--)
    {
      if (pointsEdgeLeft[i].y > 1 && abs(pointsEdgeLeft[i].y - pointsEdgeLeft[i + 1].y) < 3)
      {
        rowBreakLeftUp = i;
        counter = 0;
        counterFilter++;
      }
      else if (pointsEdgeLeft[i].y < 2 && counterFilter > 10)
      {
        counter++;
        if (counter > 5)
          return rowBreakLeftUp;
      }
    }

    return rowBreakLeftUp;
  }

  /**
   * @brief 搜索入库|赛道突变行（右下）
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
  uint16_t searchBreakRightDown(vector<POINT> pointsEdgeRight)
  {

    uint16_t rowBreakRightDown = 0;
    uint16_t counter = 0;

    for (int i = 1; i < pointsEdgeRight.size() - 50; i++) //寻找左边跳变点
    {
      if (pointsEdgeRight[i].y <= COLSIMAGE - 2)
        counter++;
      else
        counter = 0;

      if (counter > 3)
      {
        rowBreakRightDown = i - 4;
        break;
      }
    }

    return rowBreakRightDown;
  }

  /**
   * @brief 搜索出库|赛道边缘突变（左上）
   *
   * @param pointsEdgeLeft
   * @return uint16_t
   */
  uint16_t searchBreakLeft(vector<POINT> &pointsEdgeLeft)
  {
    uint16_t rowBreakLeft = pointsEdgeLeft.size() - 1;
    uint16_t counter = 0;
    for (int i = pointsEdgeLeft.size() - 1; i > 50; i--)
    {
      if (pointsEdgeLeft[i].y < 2)
      {
        counter++;
        if (counter > 3)
        {
          return i + 3;
        }
      }
      else
      {
        counter = 0;
      }
    }

    return rowBreakLeft;
  }

  /**
   * @brief 搜索出库|赛道边缘突变（右下）
   *
   * @param pointsEdgeRight
   * @return uint16_t
   */
  uint16_t searchBreakRight(vector<POINT> pointsEdgeRight)
  {
    uint16_t rowBreakRight = 0;
    uint16_t counter = 0;
    if (pointsEdgeRight.size() < 3)
      return 0;

    if (pointsEdgeRight[0].y >= COLSIMAGE - 20) //第一个点必须为右下点
    {
      for (int i = 0; i < pointsEdgeRight.size() - 50; i++) //寻找跳变点
      {
        if (pointsEdgeRight[i].y <= pointsEdgeRight[rowBreakRight].y && abs(pointsEdgeRight[i].y - pointsEdgeRight[rowBreakRight].y) < 5)
        {
          rowBreakRight = i;
          counter = 0;
        }
        else //突变点计数
        {
          counter++;
          if (counter > 3)
            return rowBreakRight;
        }
      }
      if (counter <= 3)
        return 2;
    }
    else
    {
      pointsEdgeRight[0].y = COLSIMAGE - 1;
      _Index = "x";
      return 0;
    }

    return rowBreakRight;
  }

  /**
   * @brief 搜索最佳岔路
   *
   * @param spurroad 岔路集合
   * @return POINT 岔路坐标
   */
  POINT searchBestSpurroad(vector<POINT> spurroad)
  {
    if (spurroad.size() < 1)
      return POINT(0, 0);

    vector<int> cols;
    for (int i = 0; i < spurroad.size(); i++)
    {
      cols.push_back(spurroad[i].y);
    }
    int colBest = getMiddleValue(cols); //对横坐标进行冒泡排序
    POINT spurroadBest = POINT(ROWSIMAGE, colBest);
    //搜索最佳岔路：横向中心|纵向最高
    for (int i = 0; i < spurroad.size(); i++)
    {
      if (abs(spurroadBest.y - spurroad[i].y) < 70)
      {
        if (spurroad[i].x < spurroadBest.x)
          spurroadBest.x = spurroad[i].x;
      }
    }

    return spurroadBest;
  }

  /**
   * @brief 搜索斑马线标志
   *
   * @param predict
   * @return POINT
   */
  POINT searchCrosswalkSign(vector<PredictResult> predict)
  {
    POINT crosswalk(0, 0);
    for (int i = 0; i < predict.size(); i++)
    {
      if (predict[i].label == LABEL_CROSSWALK) //标志检测
      {
        return POINT(predict[i].y + predict[i].height / 2, predict[i].x + predict[i].width / 2);
      }
    }

    return crosswalk;
  }
};
