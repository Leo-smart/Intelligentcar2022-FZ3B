
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace std;
using namespace cv;

int main()
{
    VideoWriter writer;
    int frame_fps = 20;
    int frame_width = 320;
    int frame_height = 240;

    writer = VideoWriter("../res/samples/sample.mp4", CV_FOURCC('P', 'I', 'M', '1'),
                         frame_fps, Size(frame_width, frame_height), true);
    cout << "frame_width is " << frame_width << endl;
    cout << "frame_height is " << frame_height << endl;
    cout << "frame_fps is " << frame_fps << endl;

    Mat img;
    for (int i = 1; i < 3000; i++)
    {
        string image_name = "../res/samples/train/" + to_string(i) + ".jpg";

        img = imread(image_name);
        if (!img.empty())
        {
            writer << img;
        }
        else
            break;
    }

    waitKey(0);
    return 0;
}
