#ifndef MCP4716_H_
#define MCP4716_H_

#include "ebus_i2cmaster.hpp"
#include "i2cMCmd.hpp"
#include "drv_general_io.hpp"

class mcp4716 : public i2cDevice 
{
public:
    mcp4716() = delete;
    mcp4716(ESAF::ebus_i2cmaster *i2c, uint8_t _address);
    ~mcp4716() = default;

    void set_volts(float vol);

public:
    uint8_t _addr;
    ESAF::ebus_i2cmaster *_i2c;
};

#endif
