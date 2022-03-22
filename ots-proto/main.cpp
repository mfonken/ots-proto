//
//  main.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 2/20/22.
//

#include <iostream>
#include "ots_combine_demo.hpp"

const char * file_name = "/Users/matthew/Desktop/ots-cmbine.dat";
SERCOM_Channel imu_channel = { -1, "/dev/tty.usbmodem14301", "/dev/tty.usbmodem14101", B115200, CS8, 0 };

CombineDemo::combine_utility_rate_t rates[] =
{
    { Combine::WEBCAM, 1 },
    { Combine::IMU, 1 },
//    { Combine::KINETIC, 1 }
    { Combine::COMBINE, 1 },
};

int main(int argc, const char * argv[])
{
    kinetic_config_t config = { 0 };
    Environment env("CombineDemo");
    Combine combine(&config, file_name, &imu_channel);
    CombineDemo demo(&env, &combine);
    
    demo.Init(rates);
    demo.Start();
    demo.ShowWebcam();
    
//    demo.TestIMU(1);
//    demo.TestWebcam(30);
//    demo.TestKinetic();
    return 0;
}
