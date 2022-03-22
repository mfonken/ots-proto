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
    {
        { LOCK(mutex)
            if(combine->wcu.frame.cols > 0)
            {
                imshow("figure", combine->wcu.frame);
            }
        }
        waitKey(10);
    }
}

/// TESTS
void CombineDemo::TestWebcam(int rate)
{
    pthread_mutex_t * mutex = InitUtility(Combine::WEBCAM, rate);
    Start();
    printf("demo->%p\n", mutex);
    ShowWebcam();
}

void CombineDemo::TestIMU(int rate)
{
    pthread_mutex_t * mutex = InitUtility(Combine::IMU, rate);
    Start();
    
    imu_t * imu = &combine->imu.imu;
    int i = 0;
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
    sleep(1);
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
