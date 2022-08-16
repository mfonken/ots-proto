//
//  main.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 2/20/22.
//

#include <iostream>

#include "tests.hpp"

#include "config.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/cvdef.h"
#include <opencv2/calib3d.hpp>

#define TEST

using namespace cv;

int main(int argc, const char * argv[])
{    
    Environment env("CombineDemo");
    
    Combine combine(&config, file_name, &imu_channel, &camera_intrinsics );
    CombineDemo demo(&env, &combine);
    
#ifdef TEST
//    demo.Visualizer();
    demo.TestRho(30);
//    demo.TestIMU(100);
//    demo.TestWebcam(10);
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
