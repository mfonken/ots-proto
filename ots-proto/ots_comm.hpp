//
//  ots_comm.hpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/20/22.
//

#ifndef ots_comm_hpp
#define ots_comm_hpp

#include "environment_master.hpp"

class OTSComm : public TestInterface
{
    int id;
    const std::string name;
    void init( void );
    string serialize( void );
    
    string packet;
    
public:
    pthread_mutex_t mutex;
    SerialWriter comm;
    
    OTSComm(string n = "ots-comm", SerialWriter_TYPE type = SFILE, const char * file_name = NULL);
    
    void UpdatePacket(string);
    void trigger( void );
};

#endif /* ots_comm_hpp */
