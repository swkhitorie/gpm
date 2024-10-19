#ifndef MAVLINKSTREAM_H_
#define MAVLINKSTREAM_H_

#include "mavlink_bridge_header.h"

class MavlinkStream
{
public:
    MavlinkStream(mavlink_channel_t channel) : chan(channel), rx_heartbeat_cnt(0) {}
    ~MavlinkStream() = default;

    virtual void broadcast(uint64_t time_boot_ms) = 0;
    virtual void parsing(uint8_t c) = 0;

protected:
    mavlink_channel_t chan;

    mavlink_message_t txmsg;
    mavlink_message_t rxmsg;    
    mavlink_status_t rxstatus;

    int64_t rx_heartbeat_cnt;
};

#endif
