#pragma once

/**
 * @file perspective_mapping.cpp
 * @author Leo (liaotengjun@hotmail.com)
 * @brief 图像的透视变换方法（逆/俯视）
 * @version 0.1
 * @date 2022-02-24
 * @note 通过透视变换提取摄像头上帝视角（俯视）
 * @copyright Copyright (c) 2022
 * @note IPM计算步骤：
 * [1] 设置逆透视图像的掩膜区域（mask）：包括目标变换区域和变换后的成像区域
 * [2] 求解变换矩阵和逆变矩阵
 * [3] 对图像或坐标进行变换
 */

#include <iostream>
#include <stdio.h>
#include <ctime>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

class PerspectiveMapping
{
public:
    /**
     * @brief IPM初始化
     *
     * @param origSize 输入原始图像Size
     * @param dstSize 输出图像Size
     */
    void init(const cv::Size &origSize, const cv::Size &dstSize)
    {
        // 原始域：分辨率320x240
        // The 4-points at the input image
        m_origPoints.clear();
        // [第二版无带畸变镜头参数]
        m_origPoints.push_back(Point2f(0, 214));   //左下
        m_origPoints.push_back(Point2f(319, 214)); //右下
        m_origPoints.push_back(Point2f(192, 0));   //右上
        m_origPoints.push_back(Point2f(128, 0));   //左上

        // 矫正域：分辨率320x240
        // The 4-points correspondences in the destination image
        m_dstPoints.clear();
        m_dstPoints.push_back(Point2f(100, 400)); //左下
        m_dstPoints.push_back(Point2f(220, 400)); //右下
        m_dstPoints.push_back(Point2f(220, 0));   //右上
        m_dstPoints.push_back(Point2f(100, 0));   //左上

        m_origSize = origSize;
        m_dstSize = dstSize;
        assert(m_origPoints.size() == 4 && m_dstPoints.size() == 4 && "Orig. points and Dst. points must vectors of 4 points");
        m_H = getPerspectiveTransform(m_origPoints, m_dstPoints); //计算变换矩阵 [3x3]
        m_H_inv = m_H.inv();                                      //求解逆转换矩阵

        createMaps();
    }

    /**
     * @brief 单应性反透视变换
     *
     * @param _inputImg 原始域图像
     * @param _dstImg 矫正域图像
     * @param _borderMode 矫正模式
     */
    void homographyInv(const Mat &_inputImg, Mat &_dstImg, int _borderMode)
    {
        // Generate IPM image from src
        remap(_inputImg, _dstImg, m_mapX, m_mapY, INTER_LINEAR, _borderMode); //, BORDER_CONSTANT, Scalar(0,0,0,0));
    }

    /**
     * @brief 单应性反透视变换
     *
     * @param _point 矫正域坐标
     * @return Point2d 原始域坐标
     */
    Point2d homographyInv(const Point2d &_point)
    {
        return homography(_point, m_H_inv);
    }

    /**
     * @brief 单应性反透视变换
     *
     * @param _point 矫正域坐标
     * @return Point3d 原始域坐标
     */
    Point3d homographyInv(const Point3d &_point)
    {
        return homography(_point, m_H_inv);
    }

    /**
     * @brief 单应性透视变换
     *
     * @param _point 原始域坐标
     * @return Point2d 矫正域坐标
     */
    Point2d homography(const Point2d &_point)
    {
        return homography(_point, m_H);
    }

    /**
     * @brief 单应性透视变换
     *
     * @param _point 原始域坐标
     * @param _H 转换矩阵
     * @return Point2d 矫正域坐标
     */
    Point2d homography(const Point2d &_point, const Mat &_H)
    {
        Point2d ret = Point2d(-1, -1);

        const double u = _H.at<double>(0, 0) * _point.x + _H.at<double>(0, 1) * _point.y + _H.at<double>(0, 2);
        const double v = _H.at<double>(1, 0) * _point.x + _H.at<double>(1, 1) * _point.y + _H.at<double>(1, 2);
        const double s = _H.at<double>(2, 0) * _point.x + _H.at<double>(2, 1) * _point.y + _H.at<double>(2, 2);
        if (s != 0)
        {
            ret.x = (u / s);
            ret.y = (v / s);
        }
        return ret;
    }

    /**
     * @brief 单应性透视变换
     *
     * @param _point 原始域坐标
     * @return Point3d 矫正域坐标
     */
    Point3d homography(const Point3d &_point)
    {
        return homography(_point, m_H);
    }

    /**
     * @brief 单应性透视变换
     *
     * @param _point 原始域坐标
     * @param _H 转换矩阵
     * @return Point3d
     */
    Point3d homography(const Point3d &_point, const cv::Mat &_H)
    {
        Point3d ret = Point3d(-1, -1, 1);

        const double u = _H.at<double>(0, 0) * _point.x + _H.at<double>(0, 1) * _point.y + _H.at<double>(0, 2) * _point.z;
        const double v = _H.at<double>(1, 0) * _point.x + _H.at<double>(1, 1) * _point.y + _H.at<double>(1, 2) * _point.z;
        const double s = _H.at<double>(2, 0) * _point.x + _H.at<double>(2, 1) * _point.y + _H.at<double>(2, 2) * _point.z;
        if (s != 0)
        {
            ret.x = (u / s);
            ret.y = (v / s);
        }
        else
            ret.z = 0;
        return ret;
    }

    /**
     * @brief 单应性透视变换
     *
     * @param _inputImg 原始域图像
     * @param _dstImg 矫正域图像
     */
    void homography(const Mat &_inputImg, Mat &_dstImg)
    {
        // Generate IPM image from src
        remap(_inputImg, _dstImg, m_mapX, m_mapY, CV_INTER_LINEAR); //, BORDER_CONSTANT, Scalar(0,0,0,0));
    }

    cv::Mat getH() const { return m_H; }
    cv::Mat getHinv() const { return m_H_inv; }
    void getPoints(vector<Point2f> &_origPts, vector<Point2f> &_ipmPts)
    {
        _origPts = m_origPoints;
        _ipmPts = m_dstPoints;
    }

    /**
     * @brief 绘制掩膜外框
     *
     * @param _points
     * @param _img
     */
    void drawBorder(const std::vector<cv::Point2f> &_points, cv::Mat &_img) const
    {
        assert(_points.size() == 4);

        line(_img, Point(static_cast<int>(_points[0].x), static_cast<int>(_points[0].y)), Point(static_cast<int>(_points[3].x), static_cast<int>(_points[3].y)), CV_RGB(205, 205, 0), 2);
        line(_img, Point(static_cast<int>(_points[2].x), static_cast<int>(_points[2].y)), Point(static_cast<int>(_points[3].x), static_cast<int>(_points[3].y)), CV_RGB(205, 205, 0), 2);
        line(_img, Point(static_cast<int>(_points[0].x), static_cast<int>(_points[0].y)), Point(static_cast<int>(_points[1].x), static_cast<int>(_points[1].y)), CV_RGB(205, 205, 0), 2);
        line(_img, Point(static_cast<int>(_points[2].x), static_cast<int>(_points[2].y)), Point(static_cast<int>(_points[1].x), static_cast<int>(_points[1].y)), CV_RGB(205, 205, 0), 2);

        for (size_t i = 0; i < _points.size(); i++)
        {
            circle(_img, Point(static_cast<int>(_points[i].x), static_cast<int>(_points[i].y)), 2, CV_RGB(238, 238, 0), -1);
            circle(_img, Point(static_cast<int>(_points[i].x), static_cast<int>(_points[i].y)), 5, CV_RGB(255, 255, 255), 2);
        }
    }

private:
    // Sizes
    cv::Size m_origSize;
    cv::Size m_dstSize;

    // Points
    std::vector<cv::Point2f> m_origPoints;
    std::vector<cv::Point2f> m_dstPoints;

    // Homography
    cv::Mat m_H;
    cv::Mat m_H_inv;

    // Maps
    cv::Mat m_mapX, m_mapY;
    cv::Mat m_invMapX, m_invMapY;

    void createMaps()
    {
        // Create remap images
        m_mapX.create(m_dstSize, CV_32F);
        m_mapY.create(m_dstSize, CV_32F);
        for (int j = 0; j < m_dstSize.height; ++j)
        {
            float *ptRowX = m_mapX.ptr<float>(j);
            float *ptRowY = m_mapY.ptr<float>(j);
            for (int i = 0; i < m_dstSize.width; ++i)
            {
                Point2f pt = homography(Point2f(static_cast<float>(i), static_cast<float>(j)), m_H_inv);
                ptRowX[i] = pt.x;
                ptRowY[i] = pt.y;
            }
        }

        m_invMapX.create(m_origSize, CV_32F);
        m_invMapY.create(m_origSize, CV_32F);

        for (int j = 0; j < m_origSize.height; ++j)
        {
            float *ptRowX = m_invMapX.ptr<float>(j);
            float *ptRowY = m_invMapY.ptr<float>(j);
            for (int i = 0; i < m_origSize.width; ++i)
            {
                Point2f pt = homography(Point2f(static_cast<float>(i), static_cast<float>(j)), m_H);
                ptRowX[i] = pt.x;
                ptRowY[i] = pt.y;
            }
        }
    }
};
