#include "mcp4716.h"

mcp4716::mcp4716(ESAF::ebus_i2cmaster *i2c, uint8_t _address) :
    _addr(_address), _i2c(i2c) 
{
}

void mcp4716::set_volts(float vol)
{
    if (vol > 4.98f)  vol = 4.98f;
    if (vol <= 0.0f)  vol = 0.0f;
    
    uint16_t _dac = vol * 13157.9999f;

	if (_i2c != NULL) {
		ESAF::i2cMCmd command;
        
		uint8_t buf[3];
		buf[0] = _addr >> 1;
		buf[1] = (_dac >> 8) & 0xFF;
        buf[2] = _dac & 0xFF; 
        
		command.set(_addr, &buf[0], 3, 0, 0, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->write_through(&command, 1, ESAF::EDev::NONBLOCK);
	}
}
