//
//  global_config.h
//  ots-proto
//
//  Created by Matthew Fonken on 7/16/22.
//

#ifndef global_config_h
#define global_config_h

#include "ots_combine_demo.hpp"

//#define CAM_WIDTH 1920
//#define CAM_HEIGHT 1080

const char * file_name = "/Users/matthew/Desktop/ots-combine.dat";
SERCOM_Channel imu_channel = { -1, "/dev/tty.usbmodem144401", "/dev/tty.usbmodem144201", B115200, CS8, 0.1, 0 };

static int fps = 25;

static kinetic_config_t config = { FRAME_WIDTH_BASE, FRAME_HEIGHT, FOCAL_LENGTH, D_FIXED };
static camera_intrinsics_t camera_intrinsics =
{
    {
        { 463.77108700959, 0.0, 947.8159318593995 },
        { 0.0, 463.62928545866953, 558.6748749917344 },
        { 0.0, 0.0, 1.0 },
    },
    { -0.06666608728707157, -0.015936129419735538, 0.008432133073367098, -0.0029393030590729977 }
};

static CombineDemo::combine_utility_rate_t rates[] =
{
    { Combine::WEBCAM, fps },
    { Combine::IMU, fps },
    { Combine::COMBINE, fps },
    { Combine::KINETIC, fps },
};


#endif /* global_config_h */
