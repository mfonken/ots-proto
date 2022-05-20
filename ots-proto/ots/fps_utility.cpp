//
//  fps_utility.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 4/28/22.
//

#include "fps_utility.hpp"

using namespace std;

FPSUtility::FPSUtility(string n)
: name(n)
{
    prv_t = TIMESTAMP();
}

void FPSUtility::init()
{}

void FPSUtility::trigger()
{
    double s = 0;
    for(double &ti : t) s += ti;
    rate = 1000.0 / s * (double)t.size();
    t.clear();
    
//    printf("Tick - %.2f\n", rate);
}

string FPSUtility::serialize()
{
    return to_string(Get());
}

void FPSUtility::Tick()
{
    now_t = TIMESTAMP();
    t.push_back(now_t - prv_t);
    prv_t = now_t;
//    printf("%.2fms\n", prv_t - now_t);
}

double FPSUtility::Get()
{
    return rate;
}
