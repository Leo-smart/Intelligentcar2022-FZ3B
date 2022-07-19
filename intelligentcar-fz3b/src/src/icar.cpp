/**
 * @file icar.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 智能汽车-完全模型组-顶层框架（TOP）
 * @version 0.1
 * @date 2022-03-19
 * @note 欢迎各位完全模型组的小伙伴，共同构建技术框架
 * @copyright Copyright (c) 2022
 *
 */
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <opencv2/highgui.hpp>                  //OpenCV终端部署
#include <opencv2/opencv.hpp>                   //OpenCV终端部署
#include "../include/uart.hpp"                  //串口通信驱动
#include "../include/detection.hpp"             //百度Paddle框架移动端部署
#include "../include/common.hpp"                //公共类方法文件
#include "image_preprocess.cpp"                 //图像预处理类
#include "recognition/track_recognition.cpp"    //赛道识别基础类
#include "controlcenter_cal.cpp"                //控制中心计算类
#include "motion_controller.cpp"                //智能车运动控制类
#include "recognition/cross_recognition.cpp"    //十字道路识别与路径规划类
#include "recognition/garage_recognition.cpp"   //车库及斑马线识别类
#include "recognition/freezone_recognition.cpp" //泛行区识别类
#include "detection/busy_detection.cpp"         //施工区AI检测与路径规划类
#include "detection/slope_detection.cpp"        //坡道AI检测与路径规划类

using namespace std;
using namespace cv;

void callbackSignal(int signum);
void displayWindowDetailInit(void);
std::shared_ptr<Driver> driver = nullptr; //初始化串口驱动

enum RoadType
{
    BaseHandle = 0,   //基础赛道处理
    RingHandle,       //环岛赛道处理
    CrossHandle,      //十字道路处理
    FreezoneHandle,   //泛行区处理
    GarageHandle,     //车库处理
    GasstationHandle, //加油站处理
    BusyareaHandle,   //施工区处理
    SlopeHandle       //坡道处理
};

int main(int argc, char const *argv[])
{
    std::shared_ptr<Detection> detection = nullptr; //初始化AI预测模型
    ImagePreprocess imagePreprocess;                //图像预处理类
    TrackRecognition trackRecognition;              //赛道识别
    ControlCenterCal controlCenterCal;              //车辆行驶路径拟合类
    MotionController motionController;              //运动控制
    CrossroadRecognition crossroadRecognition;      //十字道路处理
    GarageRecognition garageRecognition;            //车库识别
    FreezoneRecognition freezoneRecognition;        //泛型区识别类
    BusyareaDetection busyareaDetection;            //施工区检测
    SlopeDetection slopeDetection;                  //坡道（桥）检测类
    uint16_t counterRunBegin = 1;                   //智能车启动计数器：等待摄像头图像帧稳定
    RoadType roadType = RoadType::BaseHandle;       //初始化赛道类型
    uint16_t counterOutTrackA = 0;                  //车辆冲出赛道计数器A
    uint16_t counterOutTrackB = 0;                  //车辆冲出赛道计数器B
    uint16_t circlesThis = 1;                       //智能车当前运行的圈数
    uint16_t countercircles = 0;                    //圈数计数器

    // USB转串口的设备名为 / dev/ttyUSB0
    driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
    if (driver == nullptr)
    {
        std::cout << "Create Uart-Driver Error!" << std::endl;
        return -1;
    }
    //串口初始化，打开串口设备及配置串口数据格式
    int ret = driver->open();
    if (ret != 0)
    {
        std::cout << "Uart Open failed!" << std::endl;
        return -1;
    }

    ipm.init(Size(COLSIMAGE, ROWSIMAGE), Size(COLSIMAGEIPM, ROWSIMAGEIPM)); // IPM逆透视变换初始化

    signal(SIGINT, callbackSignal); //程序退出信号

    motionController.loadParams(); //读取配置文件
    trackRecognition.rowCutUp = motionController.params.rowCutUp;
    trackRecognition.rowCutBottom = motionController.params.rowCutBottom;
    garageRecognition.disGarageEntry = motionController.params.disGarageEntry;
    if (motionController.params.GarageEnable) //出入库使能
        roadType = RoadType::GarageHandle;    //初始赛道元素为出库

    if (motionController.params.debug) //调试模式
    {
        displayWindowDetailInit();                                                                              //显示窗口初始化
        detection = Detection::DetectionInstance("../res/samples/sample.mp4", "../res/model/mobilenet-ssd-v1"); // 视频输入源：本地视频 | AI模型文件
        printAiEnable = true;                                                                                   // 开启AI检测结果图像绘制：耗时
    }
    else //比赛模式
    {
        cout << "等待发车!!!" << endl;
        detection = Detection::DetectionInstance("/dev/video0", "../res/model/mobilenet-ssd-v1"); // 视频输入源：本地视频 | AI模型文件
        printAiEnable = false;                                                                    // 关闭AI检测结果图像绘制：节省算力

        // while (!driver->receiveStartSignal()) //串口接收下位机-比赛开始信号
        // {
        //     ;
        // }
        cout << "--------- System start!!! -------" << endl;

        for (int i = 0; i < 30; i++) // 3秒后发车
        {
            driver->carControl(0, PWMSERVOMID); //智能车停止运动|建立下位机通信
            waitKey(100);
        }
    }

    while (1)
    {
        bool imshowRec = false; //特殊赛道图像显示标志

        // 处理帧时长监测：显示单帧时长
        if (motionController.params.debug)
        {
            static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
            auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
            cout << "run frame time : " << startTime - preTime << "ms" << endl;
            preTime = startTime;
        }

        //[01] 视频源选择
        std::shared_ptr<DetectionResult> resultAI = detection->getLastFrame(); //获取Paddle多线程模型预测数据
        Mat frame = resultAI->rgb_frame;                                       //获取原始摄像头图像
        if (motionController.params.debug)
        {
            // imshow("frame", resultAI->det_render_frame);
            savePicture(resultAI->det_render_frame); //保存AI识别图像
        }
        else
        {
            if (motionController.params.saveImage) //保存原始图像
                savePicture(frame);
        }

        //[02] 图像预处理
        // Mat imgaeCorrect = imagePreprocess.imageCorrection(frame);         // 相机矫正
        Mat imgaeCorrect = frame;
        Mat imageBinary = imagePreprocess.imageBinaryzation(imgaeCorrect); // Gray

        //[03] 基础赛道识别：万物基于Track！！！！！
        trackRecognition.trackRecognition(imageBinary); //赛道线识别
        if (motionController.params.debug)
        {
            Mat imageTrack = imgaeCorrect.clone();  // RGB
            trackRecognition.drawImage(imageTrack); //图像显示赛道线识别结果
            imshow("imageTrack", imageTrack);
            savePicture(imageTrack);
        }

        // [04] 出库和入库识别与路径规划
        if (motionController.params.GarageEnable) //赛道元素是否使能
        {
            if (roadType == RoadType::GarageHandle || roadType == RoadType::BaseHandle)
            {
                countercircles++; //圈数计数
                if (countercircles > 200)
                    countercircles = 200;
                if (garageRecognition.startingCheck(resultAI->predictor_results)) //检测到起点
                {
                    busyareaDetection.reset();   //施工区检测初始化
                    freezoneRecognition.reset(); //泛行区识别复位

                    if (countercircles > 60)
                    {
                        circlesThis++;
                        countercircles = 0;
                    }
                }

                if (circlesThis >= motionController.params.circles && countercircles > 100) //入库使能：跑完N圈
                    garageRecognition.entryEnable = true;

                if (garageRecognition.garageRecognition(trackRecognition, resultAI->predictor_results))
                {
                    roadType = RoadType::GarageHandle;
                    if (garageRecognition.garageStep == garageRecognition.GarageEntryFinish) //入库完成
                    {
                        cout << ">>>>>>>   入库结束 !!!!!" << endl;
                        callbackSignal(0);
                    }
                    if (motionController.params.debug)
                    {
                        Mat imageGarage = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //初始化图像
                        garageRecognition.drawImage(trackRecognition, imageGarage);
                        imshow("imageRecognition", imageGarage);
                        imshowRec = true;
                        savePicture(imageGarage);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        //[05] 施工区检测
        if (motionController.params.BusyAreaEnable) //赛道元素是否使能
        {
            if (roadType == RoadType::BusyareaHandle || roadType == RoadType::BaseHandle)
            {
                if (busyareaDetection.busyareaDetection(trackRecognition, resultAI->predictor_results))
                {
                    if (motionController.params.debug)
                    {
                        Mat imageBusyarea = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //初始化图像
                        busyareaDetection.drawImage(trackRecognition, imageBusyarea);
                        imshow("imageRecognition", imageBusyarea);
                        imshowRec = true;
                        savePicture(imageBusyarea);

                        // 显示俯视域的赛道图像
                        // Mat imageIpm;
                        // ipm.homography(imageBusyarea, imageIpm); //图像的逆透视变换
                        // imshow("imageIpm", imageIpm);
                        // savePicture(imageIpm);
                    }
                    roadType = RoadType::BusyareaHandle;
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        // [06] 坡道（桥）检测与路径规划
        if (motionController.params.SlopEnable) //赛道元素是否使能
        {
            if (roadType == RoadType::SlopeHandle || roadType == RoadType::BaseHandle)
            {
                if (slopeDetection.slopeDetection(trackRecognition, resultAI->predictor_results))
                {
                    roadType = RoadType::SlopeHandle;
                    if (motionController.params.debug)
                    {
                        Mat imageFreezone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //初始化图像
                        slopeDetection.drawImage(trackRecognition, imageFreezone);
                        imshow("imageRecognition", imageFreezone);
                        imshowRec = true;
                        savePicture(imageFreezone);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        // [07] 泛行区检测与识别：非AI方式
        if (roadType == RoadType::FreezoneHandle || roadType == RoadType::BaseHandle)
        {
            if (freezoneRecognition.freezoneRecognition(trackRecognition, resultAI->predictor_results))
            {
                roadType = RoadType::FreezoneHandle;
                if (motionController.params.debug)
                {
                    Mat imageFreezone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //初始化图像
                    freezoneRecognition.drawImage(trackRecognition, imageFreezone);
                    imshow("imageRecognition", imageFreezone);
                    imshowRec = true;
                    savePicture(imageFreezone);
                }
            }
            else
                roadType = RoadType::BaseHandle;
        }

        // [08] 十字道路处理
        if (motionController.params.CrossEnable) //赛道元素是否使能
        {
            if (roadType == RoadType::CrossHandle || roadType == RoadType::BaseHandle)
            {
                if (crossroadRecognition.crossroadRecognition(trackRecognition))
                {
                    roadType = RoadType::CrossHandle;
                    if (motionController.params.debug)
                    {
                        Mat imageCross = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //初始化图像
                        crossroadRecognition.drawImage(trackRecognition, imageCross);
                        imshow("imageRecognition", imageCross);
                        imshowRec = true;
                        savePicture(imageCross);
                    }
                }
                else
                    roadType = RoadType::BaseHandle;
            }
        }

        // [09] 控制中心计算
        if (trackRecognition.pointsEdgeLeft.size() < 30 && trackRecognition.pointsEdgeRight.size() < 30 && roadType != RoadType::FreezoneHandle && roadType != RoadType::SlopeHandle) //防止车辆冲出赛道
        {
            counterOutTrackA++;
            counterOutTrackB = 0;
            if (counterOutTrackA > 20)
                callbackSignal(0);
        }
        else
        {
            counterOutTrackB++;
            if (counterOutTrackB > 50)
            {
                counterOutTrackA = 0;
                counterOutTrackB = 50;
            }
        }
        controlCenterCal.controlCenterCal(trackRecognition); //根据赛道边缘信息拟合运动路径（控制中心）

        // [10] 运动控制
        if (counterRunBegin > 20) //智能车启动延时：前几场图像+AI推理不稳定
        {
            //智能汽车方向控制
            motionController.pdController(controlCenterCal.controlCenter); // PD控制器姿态控制

            //智能汽车速度控制
            switch (roadType)
            {
            case RoadType::FreezoneHandle:                                               //泛行区处理速度
                if (motionController.params.FreezoneEnable)                              // AI区域的速度
                    motionController.motorSpeed = motionController.params.speedFreezone; //匀速控制
                else                                                                     //非AI区域的速度
                    motionController.speedController(true, controlCenterCal);            //变加速控制
                break;
            case RoadType::GasstationHandle:                                        //加油站速度
                motionController.motorSpeed = motionController.params.speedGasBusy; //匀速控制
                break;
            case RoadType::BusyareaHandle:                                          //施工区速度
                motionController.motorSpeed = motionController.params.speedGasBusy; //匀速控制
                break;
            case RoadType::SlopeHandle:                                          //坡道速度
                motionController.motorSpeed = motionController.params.speedSlop; //匀速控制
                break;
            default:                                                      //基础巡线 | 十字 |环岛速度
                motionController.speedController(true, controlCenterCal); //变加速控制
                break;
            }

            if (!motionController.params.debug) //调试模式下不控制车辆运动
            {
                driver->carControl(motionController.motorSpeed, motionController.servoPwm); //串口通信，姿态与速度控制
            }
        }
        else
            counterRunBegin++;

        // [11]调试模式下图像显示和存图
        if (motionController.params.debug)
        {
            controlCenterCal.drawImage(trackRecognition, imgaeCorrect);
            switch (roadType)
            {
            case RoadType::BaseHandle:                                                                                             //基础赛道处理
                putText(imgaeCorrect, "[1] Track", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::RingHandle:                                                                                            //环岛赛道处理
                putText(imgaeCorrect, "[1] Ring", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::CrossHandle:                                                                                            //十字道路处理
                putText(imgaeCorrect, "[1] Cross", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::FreezoneHandle:                                                                                            //泛行区处理
                putText(imgaeCorrect, "[1] Freezone", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::GarageHandle:                                                                                            //车库处理
                putText(imgaeCorrect, "[1] Garage", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::GasstationHandle:                                                                                            //加油站处理
                putText(imgaeCorrect, "[1] Gasstation", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::BusyareaHandle:                                                                                            //施工区处理
                putText(imgaeCorrect, "[1] Busyarea", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            case RoadType::SlopeHandle:                                                                                           //坡道处理
                putText(imgaeCorrect, "[1] Slop", Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA); //显示赛道识别类型
                break;
            }

            putText(imgaeCorrect, "v: " + formatDoble2String(motionController.motorSpeed, 2), Point(COLSIMAGE - 60, 80), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1); //车速

            string str = to_string(circlesThis) + "/" + to_string(motionController.params.circles);
            putText(imgaeCorrect, str, Point(COLSIMAGE - 50, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(0, 255, 0), 1, CV_AA); //显示圈数
            if (!imshowRec)                                                                                                                    //保持调试图像存储顺序和显示一致性
            {
                Mat imageNone = Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); //初始化图像
                imshow("imageRecognition", imageNone);
                savePicture(imageNone);
            }
            imshow("imageControl", imgaeCorrect);
            savePicture(imgaeCorrect);

            char c = waitKey(10);
        }
    }

    return 0;
}

/**
 * @brief OpenCV图像显示窗口初始化（详细参数/Debug模式）
 *
 */
void displayWindowDetailInit(void)
{
    //[1] 二值化图像：Gray
    string windowName = "imageTrack";
    cv::namedWindow(windowName, WINDOW_NORMAL); //图像名称
    cv::resizeWindow(windowName, 320, 240);     //分辨率
    cv::moveWindow(windowName, 10, 10);         //布局位置

    //[2] 赛道边缘图像：RGB
    windowName = "imageRecognition";
    cv::namedWindow(windowName, WINDOW_NORMAL); //图像名称
    cv::resizeWindow(windowName, 320, 240);     //分辨率
    cv::moveWindow(windowName, 10, 320);        //布局位置

    //[3] 原始图像/矫正后：RGB
    windowName = "imageControl";
    cv::namedWindow(windowName, WINDOW_NORMAL); //图像名称
    cv::resizeWindow(windowName, 640, 480);     //分辨率
    cv::moveWindow(windowName, 350, 20);        //布局位置
}

/**
 * @brief 系统信号回调函数：系统退出(按键ctrl+c)
 *
 * @param signum 信号量
 */
void callbackSignal(int signum)
{
    driver->carControl(0, PWMSERVOMID); //智能车停止运动
    cout << "====System Exit!!!  -->  CarStopping! " << signum << endl;
    exit(signum);
}