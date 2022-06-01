//
//  fps_utility.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 4/28/22.
//

#include "fps_utility.hpp"

using namespace std;

FPSUtility::FPSUtility(int n_samples, string name)
: n_samples(n_samples), name(name), t(n_samples)
{
    prv_t = TIMESTAMP(TIME_MS);
}

void FPSUtility::init()
{}

void FPSUtility::trigger()
{
    double s = 0;
    double n = 0;
    for(double &ti : t)
    {
        n += ti > 0 ? 1 : 0;
        s += ti;
        ti = 0;
    }
    rate = 1000.0 / s * n;   
//    printf("Tick - %.2f\n", rate);
}

string FPSUtility::serialize()
{
    return to_string(Get());
}

void FPSUtility::Tick()
{
    now_t = TIMESTAMP(TIME_MS);
    t[i++] = (now_t - prv_t);
    if(i >= n_samples) i = 0;
    prv_t = now_t;
}

double FPSUtility::Get()
{
    return rate;
}
