//
//  ots_combine_demo.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/12/22.
//

#ifndef ots_combine_demo_hpp
#define ots_combine_demo_hpp

#include "ots_combine.hpp"
#include "fps_utility.hpp"
#include "rho_drawer.hpp"
#include <opencv2/viz/types.hpp>
#include <opencv2/calib3d.hpp>

class CombineDemo
{
    Combine * combine;
    Environment * env;
    
    FPSUtility fps;
    
    char key_;
    
public:    
    typedef struct
    {
        Combine::combine_utility_e name;
        pthread_mutex_t mutex;
    } utility_mutex_t;
    
    typedef struct
    {
        Combine::combine_utility_e name;
        int rate;
    } combine_utility_rate_t;
    
    CombineDemo(Environment * env, Combine* combine);
    void Init(combine_utility_rate_t rates[]);
    void Start();
    void Pause();
    pthread_mutex_t* InitUtility(Combine::combine_utility_e name, int rate);
    
    // Handlers
    void HandleKey(char k);
    
    // Show
    void ShowWebcam();
    void ShowFrame(bool = false);
    
    // Tests
    void TestRho(int rate = 5);
    void TestWebcam(int rate = 5);
    void TestIMU(int rate = 5);
    void TestKinetic(int rate = 5);
    void TestTracker(int rate = 5);
    
    void Record(int seconds, int webcam_rate, int imu_rate, string path="~/Desktop");
    void OnFrame(cv::Mat frame, double timestamp_ms);
    
    void Visualizer(int rate = 10);
};

#endif /* ots_combine_demo_hpp */
