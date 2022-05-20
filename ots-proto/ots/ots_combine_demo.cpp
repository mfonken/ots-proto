//
//  ots_combine_demo.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/12/22.
//

#include "ots_combine_demo.hpp"
#include "unfisheye.hpp"

using namespace cv;

CombineDemo::CombineDemo(Environment * env, Combine* combine)
{
    this->env = env;
    this->combine = combine;
    
    this->env->addTest( &fps, 1.0 );
}

void CombineDemo::Init(combine_utility_rate_t rates[Combine::NUM_UTILITIES])
{
    for(int i = 0; i < Combine::NUM_UTILITIES; i++)
    {
        InitUtility(rates[i].name, rates[i].rate);
    }
}

void CombineDemo::Start()
{
    env->start();
}

void CombineDemo::Pause()
{
    env->pause();
}

pthread_mutex_t* CombineDemo::InitUtility(Combine::combine_utility_e name, int rate)
{
    TestInterface* utility = combine->GetUtility(name);
    if(utility == NULL) return NULL;
    
    env->addTest(utility, rate);
    return &utility->mutex;
}

void CombineDemo::HandleKey(char k)
{
    switch(k)
    {
        case ' ':
            OrienterFunctions.Tare( &combine->kin.orienter);
            break;
        default:
            break;
    }
}

void CombineDemo::ShowWebcam()
{
    while(true)
    { LOCK(&combine->wcu.mutex)
        if(!combine->wcu.frame.empty())
            imshow("figure", combine->wcu.frame);
        waitKey(10);
    }
}

void CombineDemo::ShowFrame(bool show_processed)
{
    const double scale = 0.75;
    while(true)
    {
        if((show_processed && combine->new_processed_frame) || (!show_processed && combine->new_frame))
        { LOCK(&combine->frame_mutex)
            Mat m;
            if(!combine->frame.empty())
            {
                fps.Tick();
                resize(combine->frame, m, Size(), scale, scale, INTER_LINEAR);
                putText(m, to_string((int)fps.Get()), Point(3, 13), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 100));
                
//                Rect crop(m.cols/2 - m.rows/2, 0, m.rows, m.rows);
//                Mat M(m, crop);
//                Mat U = Mat(M.rows, M.cols, M.type(), Scalar(255, 255, 255));
//                invfisheye(M, U);
                imshow("Demo", m);
            }
        }
        char k = waitKey(10);
        if(k > 0 && k != key_) HandleKey(k);
        key_ = k;
    }
}

/// TESTS
void CombineDemo::TestRho(int rate)
{
    InitUtility(Combine::WEBCAM, rate);
    Start();
//    int n = 26;
    while(true)
    {
        { //LOCK(mutex)
            Mat m; //cv::imread("/Users/matthew/Desktop/PersonalResources/TestImages/frames/ellipse/" + to_string(n++) + ".png");//
            threshold( combine->wcu.frame, m, IMAGE_THRESHOLD, 255, 0 );
            if(m.cols > 0)
            {
                combine->det.perform(m);
                combine->det.draw(m);
                imshow("figure", m);
            }
        }
        waitKey(1000 / rate);
//        if(n > 26) n = 0;
    }
}

void CombineDemo::TestWebcam(int rate)
{
    pthread_mutex_t * mutex = InitUtility(Combine::WEBCAM, rate);
    Start();
    printf("demo->%p\n", mutex);
    ShowFrame();
}

void CombineDemo::TestIMU(int rate)
{
    pthread_mutex_t * mutex = InitUtility(Combine::IMU, rate);
    Start();
    
    imu_t * imu = &combine->imu.imu;
//    int i = 0;
    while(true)
    {
        { LOCK(mutex)
            printf("p%.2f r%.2f y%.2f\n", imu->pitch, imu->roll, imu->yaw);
        }
        waitKey(1000 / rate);
        
//        if(i++ > rate)
//        {
//            Pause();
//            break;
//        }
    }
//    sleep(1);
}

void CombineDemo::TestKinetic(int rate)
{
    InitUtility(Combine::IMU, rate*2);
    InitUtility(Combine::KINETIC, rate);
    Start();
//    vec3_t g = { 0, 0, -1 };
    int i = 0;
    while(true)
    {
        vec3_t n, a;
        { LOCK(&combine->imu.mutex)
            IMUUtility::imu_data_t imu_data = combine->imu.FetchIMUData();
            n = { imu_data.accel[0], imu_data.accel[1], imu_data.accel[2] };
            a = { imu_data.roll, imu_data.pitch, imu_data.yaw };
//            g = { imu_data.gravity[0], imu_data.gravity[1], imu_data.gravity[2] };
        }
        { LOCK(&combine->kin.orienter_data_mutex);
            OrienterFunctions.Update( &combine->kin.orienter, &a );
            ang3_t * r = &combine->kin.orienter.rotation;
            ang3_t * rr = &combine->kin.orienter.rotation_raw;
            LOG_KU(DEBUG_2, "<%.2f, %.2f, %.2f> | <%.2f, %.2f, %.2f>\n", rr->x, rr->y, rr->z, r->x, r->y, r->z);
        }
        waitKey(1000 / rate);
        if( i++ % 5 == 0 )
        {
//            LOG_KU(DEBUG_2, "Tared: <%.2f, %.2f, %.2f>\n", g.i, g.j, g. k);
            OrienterFunctions.Tare( &combine->kin.orienter );//, &g );
        }
    }
}

void CombineDemo::TestTracker(int rate)
{
    vector<Point2f> pts = { Point2f(0, 0) };
    combine->det.tracker.Update(pts);
}
