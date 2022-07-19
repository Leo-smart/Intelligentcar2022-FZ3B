#pragma once

#include "model_config.hpp"
#include "preprocess.hpp"
#include <paddle_api.h>

struct PredictResult
{
  int type;
  std::string label;
  float score;
  int x;
  int y;
  int width;
  int height;
};

class Predictor
{
private:
  std::string _config_path;
  std::shared_ptr<ModelConfig> _model_config;
  std::shared_ptr<PaddlePredictor> _predictor;

public:
  Predictor(std::string config_path) : _config_path(config_path){};
  ~Predictor(){};

  int init()
  {
    _model_config = std::make_shared<ModelConfig>(_config_path);
    if (_model_config == nullptr)
    {
      std::cout << "Create Model config failed, config path: " << _config_path
                << std::endl;

      return -1;
    }
    std::vector<Place> valid_places({
        Place{TARGET(kFPGA), PRECISION(kFP16), DATALAYOUT(kNHWC)},
        Place{TARGET(kHost), PRECISION(kFloat)},
        Place{TARGET(kARM), PRECISION(kFloat)},
    });

    paddle::lite_api::CxxConfig config;

    if (_model_config->is_combined_model)
    {
      config.set_model_file(_model_config->model_file);
      config.set_param_file(_model_config->params_file);
    }
    else
    {
      config.set_model_dir(_model_config->model_params_dir);
    }

    config.set_valid_places(valid_places);

    _predictor = paddle::lite_api::CreatePaddlePredictor(config);
    if (!_predictor)
    {
      std::cout << "Error: CreatePaddlePredictor Failed." << std::endl;
      return -1;
    }
    std::cout << "Predictor Init Success !!!" << std::endl;
    return 0;
  };

  std::vector<PredictResult> run(cv::Mat &inputFrame)
  {
    std::vector<PredictResult> predict_ret;

    auto input = _predictor->GetInput(0);
    input->Resize(
        {1, 3, _model_config->input_height, _model_config->input_width});

    fpga_preprocess(inputFrame, _model_config, input);

    if (_model_config->is_yolo)
    {
      auto img_shape = _predictor->GetInput(1);
      img_shape->Resize({1, 2});
      auto *img_shape_data = img_shape->mutable_data<int32_t>();
      img_shape_data[0] = inputFrame.rows;
      img_shape_data[1] = inputFrame.cols;
    }

    _predictor->Run();

    auto output = _predictor->GetOutput(0);
    float *result_data = output->mutable_data<float>();
    int size = output->shape()[0];

    for (int i = 0; i < size; i++)
    {
      float *data = result_data + i * 6;
      float score = data[1];
      if (score < _model_config->threshold)
      {
        continue;
      }
      PredictResult r;
      r.type = (int)data[0];
      r.score = score;
      if (_model_config->is_yolo)
      {
        r.x = data[2];
        r.y = data[3];
        r.width = data[4] - r.x;
        r.height = data[5] - r.y;
      }
      else
      {
        r.x = data[2] * inputFrame.cols;
        r.y = data[3] * inputFrame.rows;
        r.width = data[4] * inputFrame.cols - r.x;
        r.height = data[5] * inputFrame.rows - r.y;
      }
      predict_ret.push_back(r);
    }
    return predict_ret;
  };

  void render(cv::Mat &inputFrame, std::vector<PredictResult> &results)
  {
    for (int i = 0; i < results.size(); ++i)
    {
      PredictResult r = results[i];
      _boundaryCorrection(r, inputFrame.cols, inputFrame.rows);
      if (r.type >= 0 /*0:背景*/ && r.type < _model_config->labels.size())
      {
        cv::Point origin(r.x, r.y);
        std::string label_name = _model_config->labels[r.type];
        cv::putText(inputFrame, label_name, origin, cv::FONT_HERSHEY_PLAIN, 1,
                    cv::Scalar(255, 0, 0, 255), 1);
        cv::Rect rect(r.x, r.y, r.width, r.height);
        cv::rectangle(inputFrame, rect, Scalar(0, 0, 224), 2);

        std::string score_label = std::to_string(r.score);

        cv::putText(inputFrame, score_label, Point(r.x, r.y + 15),
                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0, 255), 1);

        std::string point_string =
            "[" + std::to_string(r.x) + ":" + std::to_string(r.y) + "]";
        cv::putText(inputFrame, point_string, Point(r.x, r.y + 25),
                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0), 1);
      }
    }
  }
  void printer(cv::Mat &inputFrame, std::vector<PredictResult> results)
  {

    for (int i = 0; i < results.size(); ++i)
    {
      PredictResult r = results[i];
      if (_model_config->labels.size() > 0)
      {
        std::cout << "label:" << _model_config->labels[r.type] << ",";
      }
      std::cout << "index:" << r.type << ",score:" << r.score << " loc:";
      std::cout << r.x << "," << r.y << "," << r.width << "," << r.height
                << std::endl;
    }
  }
  std::string getLabel(int type)
  {
    if ((type < 0) || (type > _model_config->labels.size()))
    {
      std::cout << "Error: Predictor Label [" << type << "] is error."
                << std::endl;
      exit(-1);
    }
    return _model_config->labels[type];
  };

  void transmitLabels(std::vector<PredictResult> &predicts)
  {
    if (_model_config->labels.size() > 0)
    {
      for (int i = 0; i < predicts.size(); ++i)
      {
        predicts[i].label = _model_config->labels[predicts[i].type];
      }
    }
  };

private:
  void _boundaryCorrection(PredictResult &r, int width_range,
                           int height_range)
  {
#define MARGIN_PIXELS (2)
    r.width = (r.width > (width_range - r.x - MARGIN_PIXELS))
                  ? (width_range - r.x - MARGIN_PIXELS)
                  : r.width;
    r.height = (r.height > (height_range - r.y - MARGIN_PIXELS))
                   ? (height_range - r.y - MARGIN_PIXELS)
                   : r.height;

    r.x = (r.x < MARGIN_PIXELS) ? MARGIN_PIXELS : r.x;
    r.y = (r.y < MARGIN_PIXELS) ? MARGIN_PIXELS : r.y;
  }
};