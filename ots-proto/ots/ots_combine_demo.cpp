//
//  ots_combine_demo.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/12/22.
//

#include "ots_combine_demo.hpp"

using namespace cv;

CombineDemo::CombineDemo(Environment * env, Combine* combine)
{
    this->env = env;
    this->combine = combine;
    
    this->env->addTest( &fps, FPS_RATE );
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
                putText(m, to_string(fps.Get()), Point(3, 13), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 100));
                imshow("figure", m);
            }
        }
        waitKey(1);
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
    pthread_mutex_t * mutex = InitUtility(Combine::KINETIC, rate);
    Start();
    while(true)
    {
        { LOCK(mutex)
            printf("\n");
        }
    }
}

void CombineDemo::TestTracker(int rate)
{
    vector<Point2f> pts = { Point2f(0, 0) };
    combine->det.tracker.Update(pts);
}
