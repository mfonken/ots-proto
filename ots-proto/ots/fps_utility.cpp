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
{
    
}

void FPSUtility::trigger()
{
    double s = 0;
    for(double &ti : t) s += ti;
    rate = (int)(1000.0 / s * (double)t.size());
    t.clear();
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
}

int FPSUtility::Get()
{
    return rate;
}
