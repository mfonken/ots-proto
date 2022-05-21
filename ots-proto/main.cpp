//
//  main.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 2/20/22.
//

#include <iostream>
#include "ots_combine_demo.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cvdef.h"

#include "kalman2d.h"

//#define TEST

using namespace cv;

const char * file_name = "/Users/matthew/Desktop/ots-cmbine.dat";
SERCOM_Channel imu_channel = { -1, "/dev/tty.usbmodem144201", "/dev/tty.usbmodem14401", B115200, CS8, 0.1, 0 };

CombineDemo::combine_utility_rate_t rates[] =
{
    { Combine::WEBCAM, 30 },
    { Combine::IMU, 30 },
    { Combine::COMBINE, 30 },
    { Combine::KINETIC, 5 },
};

void show( cv::Mat m )
{
    while(true)
    {
        imshow( "", m );
        waitKey(10);
    }
}


int main(int argc, const char * argv[])
{
//    TrackerUtility tr;
//    vector<Point2f> pts = { Point2f(0, 0), Point2f(10, 0) };
//    tr.UpdateTrack(pts);
//    pts[0] = Point2f(0.1, 0.1); pts[1] = Point2f(10.1, 0.1);
//    tr.UpdateTrack(pts);
    
    Environment env("CombineDemo");
    kinetic_config_t config = { 1920, 1080, FOCAL_LENGTH, D_FIXED };
    Combine combine(&config, file_name, &imu_channel);
    CombineDemo demo(&env, &combine);
    
#ifndef TEST
    demo.Init(rates);
    demo.Start();
    demo.ShowFrame(true);
#else
//    demo.TestRho(10);
//    demo.TestIMU(1);
//    demo.TestWebcam(30);
    demo.TestKinetic(20);
//    demo.TestTracker();
#endif
    
    return 0;
}
