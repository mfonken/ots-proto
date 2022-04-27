//
//  ots_combine_demo.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/12/22.
//

#ifndef ots_combine_demo_hpp
#define ots_combine_demo_hpp

#define IMAGE_THRESHOLD 165

#include "ots_combine.hpp"

class CombineDemo
{
    Combine * combine;
    Environment * env;
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
    
    void ShowWebcam();
    
    void TestRho(int rate = 5);
    void TestWebcam(int rate = 5);
    void TestIMU(int rate = 5);
    void TestKinetic(int rate = 5);
    void TestTracker(int rate = 5);
};

#endif /* ots_combine_demo_hpp */
