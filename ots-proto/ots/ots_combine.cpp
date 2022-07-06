//
//  ots_combine.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/20/22.
//

#include "ots_combine.hpp"

using namespace cv;
using namespace std::placeholders;

Combine::Combine(kinetic_config_t * config, const char * file_name, SERCOM_Channel * imu_channel, camera_intrinsics_t * camera_intrinsics )
:   name("combine"),
    kin(config),
    comm(SFILE, file_name),
    imu(imu_channel),
    new_frame(false),
    new_processed_frame(false),
    wcu("webcam", camera_intrinsics, 1),
    det(camera_intrinsics)
{
    if( pthread_mutex_init(&frame_mutex, NULL) != 0 )
        printf( "mutex init failed\n" );
    
    wcu.OnFrame = std::bind(&Combine::OnFrame, this, _1, _2);
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
    LOG_CMB(DEBUG_1, "Initialized.\n");
}

string Combine::serialize( void )
{
    return this->name;
}

void Combine::UpdateIMUData()
{
    IMUUtility::imu_data_t imu_data = imu.FetchIMUData();
    vec3_t n = { imu_data.accel[0], imu_data.accel[1], imu_data.accel[2] };
    vec3_t a = { imu_data.pitch, imu_data.roll, imu_data.yaw };
//    quaternion_t o;
//    Quaternion.fromEuler( &a, &o );
    kin.UpdateIMUData( &n, &a );
    
//    LOG_CMB(DEBUG_2, "Grav: <%.2f, %.2f, %.2f> | Ori: <%.2f, %.2f, %.2f>\n", n.i, n.j, n.k, a.x, a.y, a.z);
}

void Combine::UpdatePointData()
{
    kpoint_t A = { 0 }, B = { 0 };
    { LOCK(&det.pts_mutex)
//        printf("UpdatePointData: %d\n", det.pts.size());
        vector<Point2f> pts = det.pts;
        if(pts.size() < 2) return;
        A.x = pts[0].x;
        A.y = pts[0].y;
        B.x = pts[1].x;
        B.y = pts[1].y;
//        unfisheyePixel(pts[0].x, pts[0].y, wcu.intrinsics, 3, &A.x, &A.y );
//        unfisheyePixel(pts[1].x, pts[1].y, wcu.intrinsics, 3, &B.x, &B.y );
    }
    kin.UpdatePointData(&A, &B);
    
//    LOG_CMB(DEBUG_2, "A: (%.2f, %.2f) | B: (%.2f, %.2f)\n", A.x, A.y, B.x, B.y);
}

void Combine::trigger()
{
    LOG_CMB(DEBUG_1, "trigger\n");

    // Update detection
    if(new_frame)
    {
//        LOCK(&frame_mutex)
        det.perform( frame );
        det.draw( frame );
        new_frame = false;
        new_processed_frame = true;
//        printf("\n");
    }
    
    // Update kinetic
    { LOCK(&mutex)
        UpdateIMUData();
        UpdatePointData();
    
        kin.trigger();
//
//        string packet = "";
//        comm.write(packet);
        
    }
}

void Combine::OnFrame(Mat m, double t_ns)
{
//    LOCK(&frame_mutex)
    LOG_CMB(DEBUG_1, "frame\n");
    frame = m.clone();
    new_processed_frame = false;
    new_frame = true;
}
