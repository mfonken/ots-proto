//
//  ots_combine.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/20/22.
//

#ifndef ots_combine_hpp
#define ots_combine_hpp

#include "utility_master.h"
#include "kinetic_master.h"
#include "webcam_utility.hpp"
#include "timestamp.h"
#include "imu_utility.hpp"
#include "kinetic_utility.hpp"
#include "ots_comm.hpp"

#define DEBUG_CMB

#ifdef DEBUG_CMB
#define LOG_CMB(L, ...) LOG(L, "<CombineUtility> " __VA_ARGS__)
#else
#define LOG_CMB(L, ...)
#endif

class Combine : public TestInterface
{
    int id;
    const std::string name;
    void init( void );
    string serialize( void );
    
public:
    typedef enum
    {
        IMU = 0,
        WEBCAM,
        COMBINE,
        NUM_UTILITIES,
        KINETIC,
    } combine_utility_e;
    
    typedef struct
    {
        kpoint_t a;
        kpoint_t b;
    } kpoints_t;
    
    void trigger( void );
    pthread_mutex_t mutex;
    
    SerialWriter comm;
    WebcamUtility wcu;
    IMUUtility imu;
    KineticUtility kin;
    
    Combine(kinetic_config_t * config, const char * file_name, SERCOM_Channel * imu_channel );

    TestInterface* GetUtility(combine_utility_e name);
    void UpdateIMUData();
    void UpdatePointData();
};
#endif /* ots_combine_hpp */
