//
//  Sweeper.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 2/18/18.
//  Copyright © 2018 Marbl. All rights reserved.
//

#ifndef sweeper_hpp
#define sweeper_hpp

#include <stdio.h>
#include <string>
#include "environment_master.hpp"

typedef enum
{
    RIGHT = 1,
    LEFT  = -1
} direction;

void toggle( direction * d )
{
    if(*d == RIGHT) *d = LEFT;
    else if(*d == LEFT) *d = RIGHT;
}

class Sweeper : public TestInterface
{
    /* Interface Concretions */
public:
    int id;
    std::string name;
    void  init( void ) {};
    void  trigger( void )
    {
        value += step * dir;
        counter += dir;
        if(counter >= steps || counter <= 0) toggle(&dir);
        inc = step * dir;
    };
    string serialize( void ) { return name; };
    
    int counter;
    double steps, step, start, stop, range, value, inc;
    direction dir;
    Sweeper(std::string n, int s, int stt, int stp)
    {
        name = n;
        id = 0;
        value = stt;
        steps = s;
        if(stt < stp)
        {
            dir = RIGHT;
            counter = 0;
        }
        else
        {
            dir = LEFT;
            counter = s;
        }
        range = fabs(stt-stp);
        step = ((double)range)/((double)steps);
        inc = step * dir;
    };
};

#endif /* sweeper_h */
