#pragma once
/**
 * @file motion_controller.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 运动控制器：PD姿态控制||速度控制||读取JSON配置文件（不需要编译程序即发车）
 * @version 0.1
 * @date 2022-02-22
 * @note PD控制器要求稳定的控制周期：~40ms
 * @copyright Copyright (c) 2022
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "controlcenter_cal.cpp"

using namespace std;
using nlohmann::json;

class MotionController
{
private:
    float speedLow = 1.0;      //智能车最低速
    float speedHigh = 1.0;     //智能车最高速
    float speedDown = 0.5;     //特殊区域降速速度
    float speedFreezone = 0.8; //泛行区行驶速度
    float speedGasBusy = 1.0;  //加油站|施工区行驶速度
    float speedSlop = 1.0;     //坡道（桥）行驶速度
    float runP1 = 0.9f;        //一阶比例系数：直线控制量
    float runP2 = 0.018f;      //二阶比例系数：弯道控制量
    float runP3 = 0.013f;      //三阶比例系数：弯道控制量
    float turnP = 3.5f;        //一阶比例系数：转弯控制量
    float turnD = 3.5f;        //一阶微分系数：转弯控制量
    int counterShift = 0;      //变速计数器

public:
    bool debug = false;              //调试模式使能
    bool saveImage = false;          //存图使能
    uint16_t rowCutUp = 10;          //图像顶部切行
    uint16_t rowCutBottom = 10;      //图像顶部切行
    uint16_t servoPwm = PWMSERVOMID; //发送给舵机的PWM
    float motorSpeed = speedLow;     //发送给电机的速度
    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float speedLow = 1.0;         //智能车最低速
        float speedHigh = 1.0;        //智能车最高速
        float speedDown = 0.5;        //特殊区域降速速度
        float speedFreezone = 0.8;    //泛行区行驶速度
        float speedGasBusy = 1.0;     //加油站|施工区行驶速度
        float speedSlop = 1.0;        //坡道（桥）行驶速度
        float runP1 = 0.9;            //一阶比例系数：直线控制量
        float runP2 = 0.018;          //二阶比例系数：弯道控制量
        float runP3 = 0.0;            //三阶比例系数：弯道控制量
        float turnP = 3.5;            //一阶比例系数：转弯控制量
        float turnD = 3.5;            //一阶微分系数：转弯控制量
        bool debug = false;           //调试模式使能
        bool saveImage = false;       //存图使能
        uint16_t rowCutUp = 10;       //图像顶部切行
        uint16_t rowCutBottom = 10;   //图像顶部切行
        float disGarageEntry = 0.7;   //车库入库距离(斑马线Image占比)
        float rateTurnFreezone = 0.5; //泛行区躲避禁行标志的转弯曲率
        bool GarageEnable = true;     //出入库使能
        bool GasStationEnable = true; //加油站使能
        bool BusyAreaEnable = true;   //施工区使能
        bool SlopEnable = true;       //坡道使能
        bool FreezoneEnable = true;   //泛行区使能
        bool RingEnable = true;       //环岛使能
        bool CrossEnable = true;      //十字使能
        uint16_t circles = 2;         //智能车运行圈数

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh, speedDown, speedFreezone, speedGasBusy, speedSlop,
                                       runP1, runP2, runP3, turnP, turnD, debug, saveImage, rowCutUp, rowCutBottom, disGarageEntry,
                                       GarageEnable, GasStationEnable, BusyAreaEnable, SlopEnable, FreezoneEnable, RingEnable, CrossEnable, circles); //添加构造函数
    };

    Params params; //读取控制参数
    /**
     * @brief 姿态PD控制器
     *
     * @param controlCenter 智能车控制中心
     */
    void pdController(int controlCenter)
    {
        float error = controlCenter - COLSIMAGE / 2; //图像控制中心转换偏差
        static int errorLast = 0;                    //记录前一次的偏差
        if (abs(error - errorLast) > COLSIMAGE / 10)
        {
            error = error > errorLast ? errorLast + COLSIMAGE / 10 : errorLast - COLSIMAGE / 10;
        }

        turnP = abs(error) * runP2 + runP1;
        turnP = max(turnP, 0.2f);
        if (turnP < 0.2)
        {
            turnP = 0.2;
        }
        turnP = runP1 + ROWSIMAGE / 2 * runP2;

        int pwmDiff = (error * turnP) + (error - errorLast) * turnD;
        errorLast = error;

        servoPwm = (uint16_t)(PWMSERVOMID + pwmDiff); // PWM转换
    }

    /**
     * @brief 变加速控制
     *
     * @param enable 加速使能
     * @param control
     */
    void speedController(bool enable, ControlCenterCal control)
    {
        //控制率
        uint8_t controlLow = 0;   //速度控制下限
        uint8_t controlMid = 5;   //控制率
        uint8_t controlHigh = 10; //速度控制上限

        if (enable) //加速使能
        {
            if (control.centerEdge.size() < 10)
            {
                motorSpeed = speedLow;
                counterShift = controlLow;
                return;
            }
            if (control.centerEdge[control.centerEdge.size() - 1].x > ROWSIMAGE / 2)
            {
                motorSpeed = speedLow;
                counterShift = controlLow;
                return;
            }
            if (abs(control.sigmaCenter) < 100.0)
            {
                counterShift++;
                if (counterShift > controlHigh)
                    counterShift = controlHigh;
            }
            else
            {
                counterShift--;
                if (counterShift < controlLow)
                    counterShift = controlLow;
            }

            if (counterShift > controlMid)
                motorSpeed = speedHigh;
            else
                motorSpeed = speedLow;
        }
        else
        {
            counterShift = controlLow;
            motorSpeed = speedLow;
        }
    }

    /**
     * @brief 加载配置参数Json
     */
    void loadParams()
    {
        string jsonPath = "../src/config/motion.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good())
        {
            std::cout << "Error: Params file path:[" << jsonPath
                      << "] not find .\n";
            exit(-1);
        }

        json js_value;
        config_is >> js_value;

        try
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        this->speedLow = params.speedLow;
        this->speedHigh = params.speedHigh;
        this->speedDown = params.speedDown;
        this->speedFreezone = params.speedFreezone;
        this->speedGasBusy = params.speedGasBusy;
        this->speedSlop = params.speedSlop;
        this->runP1 = params.runP1;
        this->runP2 = params.runP2;
        this->runP3 = params.runP3;
        this->turnP = params.turnP;
        this->turnD = params.turnD;
        this->debug = params.debug;
        this->saveImage = params.saveImage;
        this->rowCutUp = params.rowCutUp;
        this->rowCutBottom = params.rowCutBottom;

        motorSpeed = speedLow;
        cout << "--- runP1:" << runP1 << " | runP2:" << runP2 << " | runP3:" << runP3 << endl;
        cout << "--- turnP:" << turnP << " | turnD:" << turnD << endl;
        cout << "--- speedLow:" << speedLow << "m/s  |  speedHigh:" << speedHigh << "m/s" << endl;
    }
};
