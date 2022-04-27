//
//  ots_combine.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/20/22.
//

#include "ots_combine.hpp"

using namespace cv;
using namespace std::placeholders;

Combine::Combine(kinetic_config_t * config, const char * file_name, SERCOM_Channel * imu_channel )
: name("combine"), kin(config), comm(SFILE, file_name), imu(imu_channel), new_frame(false), new_processed_frame(false)//, wcu("Cam", 1)
{
    if( pthread_mutex_init(&frame_mutex, NULL) != 0 )
        printf( "mutex init failed\n" );
    
    wcu.OnFrame = std::bind(&Combine::OnFrame, this, _1);
    
}

TestInterface* Combine::GetUtility(combine_utility_e name)
{
    TestInterface* utility = NULL;
    switch(name)
    {
        case COMBINE:
            utility = this;
            break;
        case WEBCAM:
            utility = &wcu;
            break;
        case IMU:
            utility = &imu;
            break;
        case KINETIC:
            utility = &kin;
            break;
        default:
            break;
    }
    return utility;
}

void Combine::init( void )
{
}

string Combine::serialize( void )
{
    return this->name;
}

void Combine::UpdateIMUData()
{
    IMUUtility::imu_data_t imu_data = imu.FetchIMUData();
    vec3_t n = { imu_data.accel[0], imu_data.accel[1], imu_data.accel[2] };
    ang3_t a = { imu_data.roll, imu_data.pitch, imu_data.yaw };
    quaternion_t o;
    Quaternion.fromEuler( &a, &o );
    kin.UpdateIMUDate( &n, &o );
}

void Combine::UpdatePointData()
{
//    kpoints_t points = rho.FetchPointData();
//    kin.UpdatePointData(points.a, points.b);
}

void Combine::trigger()
{ LOCK(&mutex)
    UpdateIMUData();
//    UpdatePointData();
//
//    kin.trigger();
    
//    LOG_CMB(DEBUG_1, "Combine::trigger\n");
    string packet = "";
    comm.write(packet);
    
    if(new_frame)
    { LOCK(&frame_mutex)
        det.perform( frame );
        det.draw( frame );
        new_frame = false;
        new_processed_frame = true;
    }
}

void Combine::OnFrame(Mat m)
{ LOCK(&frame_mutex)
    frame = m.clone();
    new_frame = true;
}
