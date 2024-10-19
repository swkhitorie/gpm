#ifndef MAVLINK_BRIDGE_HEADER_H
#define MAVLINK_BRIDGE_HEADER_H
#include "stdint.h"

#if defined(__CC_ARM) || defined(__clang__)
    /*
     * avoid MDK errors and warning
    */
    #define MAVPACKED( __Declaration__ )  __Declaration__ 
    #if defined(__CC_ARM)
        #pragma anon_unions
    #endif
    #define inline __inline
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
//#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES mavlink_send_bytes

#define MAVLINK_START_UART_SEND mavlink_start_send
#define MAVLINK_END_UART_SEND mavlink_end_send

#include "mavlink_types.h"

#ifdef __cplusplus
extern "C" {
#endif
    extern mavlink_system_t mavlink_system; 
    void mavlink_send_bytes(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);
    void mavlink_start_send(mavlink_channel_t chan, uint16_t len);
    void mavlink_end_send(mavlink_channel_t chan, uint16_t len);
#ifdef __cplusplus
}
#endif

#include "mavlink.h"
#include "mavlink_types.h"

#endif

