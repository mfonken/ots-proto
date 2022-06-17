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
#include <opencv2/calib3d.hpp>

#include "kalman2d.h"

//#define TEST

using namespace cv;

const char * file_name = "/Users/matthew/Desktop/ots-cmbine.dat";
SERCOM_Channel imu_channel = { -1, "/dev/tty.usbmodem144201", "/dev/tty.usbmodem144301", B115200, CS8, 0.1, 0 };

CombineDemo::combine_utility_rate_t rates[] =
{
    { Combine::WEBCAM, 15 },
    { Combine::IMU, 15 },
    { Combine::COMBINE, 15 },
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
    camera_intrinsics_t camera_intrinsics =
    {
        {
            { 463.77108700959, 0.0, 947.8159318593995 },
            { 0.0, 463.62928545866953, 558.6748749917344 },
            { 0.0, 0.0, 1.0 },
        },
        { -0.06666608728707157, -0.015936129419735538, 0.008432133073367098, -0.0029393030590729977 }
    };
    
    
//    Mat K = Mat(3, 3, CV_64FC1, &camera_intrinsics.K);
//    Mat D = Mat(4, 1, CV_64FC1, &camera_intrinsics.D);
//    Mat map1, map2;
//    initUndistortRectifyMap(K, D, cv::Mat(), K, Size(1920, 1080), CV_16SC2, map1, map2);
////    remap(m, u, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//    imshow("Figure", map1);
//    waitKey(10000);
    
    Combine combine(&config, file_name, &imu_channel, &camera_intrinsics);
    CombineDemo demo(&env, &combine);
    
#ifdef TEST
//    demo.TestRho(10);
//    demo.TestIMU(100);
    demo.TestWebcam(5);
//    demo.TestKinetic(20);
//    demo.TestTracker();
//    demo.Record(45, 15, 30, "/Users/matthew/Desktop/dev/cam-imu-dataset");
#else
    demo.Init(rates);
    demo.Start();
    demo.ShowFrame(true);
#endif
    
    return 0;
}
