#ifndef __EBUS_CAN_H_
#define __EBUS_CAN_H_

#include "edev.hpp"
#include "stdint.h"

namespace ESAF
{

class ebus_can : public EDev
{
public:
    ebus_can() = default;
    virtual ~ebus_can() = default;

    void configparams(uint32_t speed_Kbps)
    {
        _pspeed = speed_Kbps;
    }
    
public:
    /**
     * @brief can classic mode
     */
    uint32_t _pspeed;
};

} // namespace ESAF

#endif  // ebus_can_H
