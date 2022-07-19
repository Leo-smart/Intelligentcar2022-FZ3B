#pragma once 
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

inline cv::Mat rbg2gray(cv::Mat rgb_frame) {
  cv::Mat im_gray;
  cv::cvtColor(rgb_frame, im_gray, cv::COLOR_RGB2GRAY);
  cv::Mat frame = im_gray.isContinuous() ? im_gray : im_gray.clone();
  if (!frame.isContinuous()) {
    std::cout << "Error frame is not continuous ..." << std::endl;
    exit(-1);
  }
  return frame;
}
