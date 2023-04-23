//
//  ots_combine.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/20/22.
//

#include "ots_combine.hpp"

using namespace cv;
using namespace std::placeholders;

Combine::Combine(kinetic_config_t * config, const char * comm_data, SERCOM_Channel * imu_channel, camera_intrinsics_t * camera_intrinsics )
:   name("combine"),
    kin(config),
    comm(COMM_TYPE, comm_data),
    imu(imu_channel),
    new_frame(false),
    new_processed_frame(false),
    wcu("webcam", camera_intrinsics, CAMERA_ID, config->width, config->height),
    det(camera_intrinsics),
    rho_drawer(&RhoSystem.Variables.Utility, &rho.capture, config->width, config->height )
{
    if( pthread_mutex_init(&frame_mutex, NULL) != 0 )
        printf( "mutex init failed\n" );
    
    comm.write("Connected to ots-combine");
    wcu.OnFrame = std::bind(&Combine::OnFrame, this, _1, _2);
}

TestInterface* Combine::GetUtility(combine_utility_e name)
{
    TestInterface* utility = NULL;
    switch(name)
    {
        case COMBINE:
            utility = (TestInterface*)this;
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
        case BLOB_DET:
            utility = &det;
            break;
        default:
            break;
    }
    return utility;
}

void Combine::init( void )
{
    rho.Init(wcu.size.width, wcu.size.height);
    
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

void Combine::UpdatePointData(vector<Point2f> pts)
{
    kpoint_t A = { 0 }, B = { 0 };
    { LOCK(&det.pts_mutex)
//        printf("UpdatePointData: %d\n", det.pts.size());
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
    { LOCK(&frame_mutex)
#ifdef USE_RHO
        rho.Perform( frame );
        rho_drawer.DrawDensityGraph( frame );
        det.pts = rho.GetPredictions();
#else
//        LOCK(&frame_mutex)
        det.perform( frame );
        det.draw( frame );
#endif
        new_frame = false;
        new_processed_frame = true;
//        printf("\n");
    }
    
    // Update kinetic
    { LOCK(&mutex)
        UpdateIMUData();
        UpdatePointData(det.pts);
    
        kin.trigger();
        
        comm.write(kin.serialize(), false);
//
//        string packet = "";
//        comm.write(packet);
        
    }
}

void Combine::OnFrame(Mat m, double t_ms)
{
    LOG_CMB(DEBUG_1, "frame\n");
#ifdef IMAGE_THRESHOLD
    threshold( m, frame, IMAGE_THRESHOLD, 255, 0 );
#else
    { LOCK(&frame_mutex)
        frame = m.clone();
    }
#endif
    new_processed_frame = false;
    new_frame = true;
}
