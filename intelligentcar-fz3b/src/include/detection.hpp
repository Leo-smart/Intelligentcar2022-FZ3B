#pragma once

#include "capture.hpp"
#include "predictor.hpp"
#include "mat_util.hpp"
#include "stop_watch.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

struct DetectionResult
{
  cv::Mat det_render_frame;
  cv::Mat rgb_frame;
  std::vector<PredictResult> predictor_results;
};

class Detection
{

public:
  Detection(bool logEn = false) : _log_en(logEn) {}
  ~Detection() {}

  int init(std::string file_path, std::string model_config_path)
  {
    _is_file = true;
    _file_path = file_path;
    _capture = std::make_shared<Capture>();
    if (_capture == nullptr)
    {
      std::cout << "Capture create failed." << std::endl;
      return -1;
    }
    int ret = _capture->open(file_path);
    if (ret != 0)
    {
      std::cout << "Capture open failed." << std::endl;
      return -1;
    }
    return _init(model_config_path);
  };

  int init(int camera_index, std::string model_config_path)
  {
    _capture = std::make_shared<Capture>();
    if (_capture == nullptr)
    {
      std::cout << "Capture create failed." << std::endl;
      return -1;
    }
    int ret = _capture->open(camera_index);
    if (ret != 0)
    {
      std::cout << "Capture open failed." << std::endl;
      return -1;
    }
    return _init(model_config_path);
  };
  void start()
  {
    _thread = std::make_unique<std::thread>([this]()
                                            {
      while (1) {
        std::shared_ptr<DetectionResult> result =
            std::make_shared<DetectionResult>();

        StopWatch stop_watch_capture;
        stop_watch_capture.tic();
        result->rgb_frame = _capture->read();
        //reopen file
        if (result->rgb_frame.empty() && _is_file) {
          _capture->close();
          int ret = _capture->open(_file_path);
          if (ret != 0) {
            std::cout << "Capture open failed." << std::endl;
            return -1;
          }
          continue;
        }

        double capture_times = stop_watch_capture.toc();

        if (result->rgb_frame.empty()) {
          std::cout << "Error: Capture Get Empty Error Frame." << std::endl;
          exit(-1);
        }
        result->predictor_results = _predictor->run(result->rgb_frame);

        if(printAiEnable) //绘制AI识别结果
        {
          result->det_render_frame = result->rgb_frame.clone();
          _predictor->render(result->det_render_frame, result->predictor_results);
        }

        _predictor->transmitLabels(result->predictor_results);//标签转换
        
        //多线程共享数据传递
        std::unique_lock<std::mutex> lock(_mutex);
        _lastResult = result;
        cond_.notify_all();
        if(printAiEnable)//调试模式下降低帧率
        {
          waitKey(30);
        }
          
      } });
  }
  int stop() { return 0; }
  int deinit() { return 0; }

  std::shared_ptr<DetectionResult> getLastFrame()
  {
    std::shared_ptr<DetectionResult> ret = nullptr;
    {
      std::unique_lock<std::mutex> lock(_mutex);

      while (_lastResult == nullptr)
      {
        cond_.wait(lock);
      }
      ret = _lastResult;
      _lastResult = nullptr;
    }
    return ret;
  }

  std::string getLabel(int type) { return _predictor->getLabel(type); }

public:
  static std::shared_ptr<Detection> DetectionInstance(std::string file_path, std::string model_path)
  {
    static std::shared_ptr<Detection> detectioner = nullptr;
    if (detectioner == nullptr)
    {
      detectioner = std::make_shared<Detection>();
      int ret = detectioner->init(file_path, model_path);
      if (ret != 0)
      {
        std::cout << "Detection init error :" << model_path << std::endl;
        exit(-1);
      }
      detectioner->start();
    }
    return detectioner;
  }

  static std::shared_ptr<Detection> DetectionInstance()
  {
    static std::shared_ptr<Detection> detectioner = nullptr;
    if (detectioner == nullptr)
    {
      detectioner = std::make_shared<Detection>();
      std::string model_path = "../res/model/mobilenet-ssd";
      int ret = detectioner->init(0, model_path);
      if (ret != 0)
      {
        std::cout << "Detection init error :" << model_path << std::endl;
        exit(-1);
      }
      detectioner->start();
    }
    return detectioner;
  }

private:
  int _init(std::string model_config_path)
  {
    _predictor = std::make_shared<Predictor>(model_config_path);
    if (_predictor == nullptr)
    {
      std::cout << "Predictor Create failed." << std::endl;
      return -1;
    }
    int ret = _predictor->init();
    if (ret != 0)
    {
      std::cout << "Predictor init failed." << std::endl;
      return -1;
    }
    return 0;
  }

private:
  bool _is_file = false;
  std::string _file_path;
  bool _log_en;
  std::shared_ptr<DetectionResult> _lastResult;

  std::mutex _mutex;
  std::condition_variable cond_;

  std::unique_ptr<std::thread> _thread;

  std::shared_ptr<Capture> _capture;
  std::shared_ptr<Predictor> _predictor;
};