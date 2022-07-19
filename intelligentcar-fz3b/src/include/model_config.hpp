#pragma once
#include <vector>
#include <fstream>
#include <iostream>
#include "json.hpp"
#include <opencv2/opencv.hpp>

using nlohmann::json;

struct ModelConfig
{
  std::string model_parent_dir;

  std::string network_type;

  std::string model_file;
  std::string model_params_dir;
  std::string params_file;

  std::string format;
  uint16_t input_width;
  uint16_t input_height;

  float means[3];
  float scales[3];
  float threshold;

  bool is_yolo;
  bool is_combined_model;

  std::vector<std::string> labels;

  void assert_check_file_exist(std::string fileName, std::string modelPath)
  {
    std::ifstream infile(modelPath + fileName);
    if (!infile.good())
    {
      printf("Error!!!! ModelConfig: File %s not exit, Please Check your model "
             "path:%s .\n",
             fileName.c_str(), modelPath.c_str());
      exit(-1);
    }
  }
  ModelConfig(std::string model_path) : model_parent_dir(model_path + "/")
  {

    std::string json_config_path = model_parent_dir + "config.json";
    std::ifstream is(json_config_path);
    if (!is.good())
    {
      std::cout << "Error:ModelConfig file path:[" << json_config_path
                << "] not find ." << std::endl;
      exit(-1);
    }

    json value;
    is >> value;
    std::cout << "Config:" << value << std::endl;

    input_width = value["input_width"];
    input_height = value["input_height"];
    format = value["format"];
    std::transform(format.begin(), format.end(), format.begin(), ::toupper);

    std::vector<float> mean = value["mean"];
    for (int i = 0; i < mean.size(); ++i)
    {
      means[i] = mean[i];
    }

    std::vector<float> scale = value["scale"];
    for (int i = 0; i < scale.size(); ++i)
    {
      scales[i] = scale[i];
    }

    if (value["threshold"] != nullptr)
    {
      threshold = value["threshold"];
    }
    else
    {
      threshold = 0.5;
      std::cout << "Warnning !!!!,json key: threshold not found, default :"
                << threshold << "\n";
    }

    is_yolo = false;
    if (value["network_type"] != nullptr)
    {
      network_type = value["network_type"];
      if (network_type == "YOLOV3")
      {
        is_yolo = true;
      }
    }

    if ((value["model_file_name"] != nullptr) &&
        (value["params_file_name"] != nullptr) &&
        (value["model_dir"] == nullptr))
    {
      is_combined_model = true;
      params_file =
          model_parent_dir + value["params_file_name"].get<std::string>();
      model_file =
          model_parent_dir + value["model_file_name"].get<std::string>();
      model_params_dir = "";
    }
    else if ((value["model_file_name"] == nullptr) &&
             (value["params_file_name"] == nullptr) &&
             (value["model_dir"] != nullptr))
    {
      is_combined_model = false;
      model_params_dir =
          model_parent_dir + value["model_dir"].get<std::string>();
      params_file = "";
      model_file = "";
    }
    else
    {
      std::cout
          << "json config Error !!!! \n combined_model: need params_file_name "
             "model_file_name, separate_model: need model_dir only.\n";
      exit(-1);
    }

    if (value["labels_file_name"] != nullptr)
    {
      std::string labels_file_name = value["labels_file_name"];
      std::string label_path = model_parent_dir + labels_file_name;
      std::ifstream file(label_path);
      if (file.is_open())
      {
        std::string line;
        while (getline(file, line))
        {
          labels.push_back(line);
        }
        file.close();
      }
      else
      {
        std::cout << "Open Lable File failed, file path: " << label_path
                  << std::endl;
      }
    }

    if (is_combined_model)
    {
      assert_check_file_exist(value["model_file_name"], model_parent_dir);
      assert_check_file_exist(value["params_file_name"], model_parent_dir);
    }
    else
    {
      assert_check_file_exist(value["model_dir"], model_parent_dir);
    }

    std::cout << "Model Config Init Success !!!" << std::endl;
  }
  ~ModelConfig() {}
};