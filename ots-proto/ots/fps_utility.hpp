//
//  fps.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 4/28/22.
//

#ifndef fps_utility_hpp
#define fps_utility_hpp

#include <stdio.h>
#include <vector>

#include "environment_master.hpp"
#include "timestamp.h"

class FPSUtility : public TestInterface
{
    int id;
    const std::string name;
    void init( void );
    string serialize( void );
    
    std::vector<double> t;
    double now_t;
    double prv_t;
    int rate = 0;
    
public:
    FPSUtility(string n = "fps");
    
    void trigger();
    
    void Tick();
    int Get();
};

#endif /* fps_hpp */
