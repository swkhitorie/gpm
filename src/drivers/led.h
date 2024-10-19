
#ifndef __LED_H_
#define __LED_H_

#include "drv_general_io.hpp"

class led
{
public:
    led() = delete;
    led(ESAF::drv_general_io *io, bool lowon);
    ~led();

    void on();
    void off();
    void hz(float hz, float ms_tick);
public:
    ESAF::drv_general_io *_io;
    bool _lowon;
    float _lsttime;
};

#endif
