#include "mavlink_vehicle.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

////////////////////////////////////////////////////////////
///////////////////// LOG INTERFACE ////////////////////////
////////////////////////////////////////////////////////////
void mavlink_log_info(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, &buffer[0], 0, 0); 
    
    va_end(args); 
}

void mavlink_log_critical(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_CRITICAL, &buffer[0], 0, 0); 
    
    va_end(args); 
}

void mavlink_log_notice(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_NOTICE, &buffer[0], 0, 0); 
    
    va_end(args); 
}

void mavlink_log_debug(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_DEBUG, &buffer[0], 0, 0); 
    
    va_end(args);   
}

void mavlink_log_warning(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_WARNING, &buffer[0], 0, 0); 
    
    va_end(args); 
}

void mavlink_log_error(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_ERROR, &buffer[0], 0, 0); 
    
    va_end(args); 
}

void mavlink_log_emergency(mavlink_channel_t chan, char *fmt, ...)
{
    char buffer[50];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 50, fmt, args);
    
    mavlink_msg_statustext_send(chan, MAV_SEVERITY_EMERGENCY, &buffer[0], 0, 0); 
    
    va_end(args);
}
