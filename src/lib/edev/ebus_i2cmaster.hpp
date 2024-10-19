#ifndef __EBUS_I2CMASTER_H_
#define __EBUS_I2CMASTER_H_

#include "edev.hpp"

namespace ESAF
{

class ebus_i2cmaster : public EDev
{
public:
    ebus_i2cmaster() = default;
    virtual ~ebus_i2cmaster() = default;
};

} // namespace ESAF

#endif  // ebus_i2cmaster_H
