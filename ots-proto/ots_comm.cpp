//
//  ots_comm.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/20/22.
//

#include "ots_comm.hpp"

OTSComm::OTSComm(std::string name, SerialWriter_TYPE type, const char * file_name)
: name(name), comm(type, file_name)
{
}

void OTSComm::UpdatePacket(string packet)
{ LOCK(&mutex)
    this->packet = packet;
}

void OTSComm::init()
{
    this->packet = "";
}

void OTSComm::trigger()
{ LOCK(&mutex)
    comm.write(this->packet);
}

string OTSComm::serialize()
{
    return this->name;
}
