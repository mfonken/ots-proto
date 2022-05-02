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

using namespace cv;

const char * file_name = "/Users/matthew/Desktop/ots-cmbine.dat";
SERCOM_Channel imu_channel = { -1, "/dev/tty.usbmodem144201", "/dev/tty.usbmodem14101", B115200, CS8, 0 };

CombineDemo::combine_utility_rate_t rates[] =
{
    { Combine::WEBCAM, 30 },
    { Combine::IMU, 30 },
//    { Combine::KINETIC, 1 }
    { Combine::COMBINE, 30 },
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
    Environment env("CombineDemo");
    
    kinetic_config_t config = { 0 };
    Combine combine(&config, file_name, &imu_channel);
    CombineDemo demo(&env, &combine);
    
    demo.Init(rates);
    demo.Start();
    demo.ShowFrame(true);
    
//    demo.TestRho(10);
//    demo.TestIMU(1);
//    demo.TestWebcam(30);
//    demo.TestKinetic();
//    demo.TestTracker();
    return 0;
}
