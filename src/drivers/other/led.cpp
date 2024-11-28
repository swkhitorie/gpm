#include "led.h"

led::led(ESAF::drv_general_io *io, bool lowon) : _io(io), _lowon(lowon), _lsttime(0.0f) {}
led::~led() {}

void led::on()
{
    if(_lowon)
        _io->set(false);
    else
        _io->set(true);
}

void led::off()
{
    if(_lowon)
        _io->set(true);
    else
        _io->set(false);
}

void led::hz(float hz, float ms_tick)
{
    if (hz < 0.001f || hz > 1000) return;
    
    float curTime = ((float)ms_tick) / 1000.0f;
    
    if (curTime - _lsttime > 0.5f / hz) {
        if (_io->get())
            _io->set(false);
        else
            _io->set(true);
        _lsttime = curTime;
    }    
}

