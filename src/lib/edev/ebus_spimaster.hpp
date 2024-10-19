#ifndef __EBUS_SPIMASTER_H_
#define __EBUS_SPIMASTER_H_

#include "edev.hpp"
#include "stdint.h"

namespace ESAF
{

class ebus_spimaster : public EDev
{
public:
    enum ESPIMODE {
        ESPIMODE_1,	   /*!< sck idle -> low level, data sample time -> rising */
        ESPIMODE_2,    /*!< sck idle -> low level, data sample time -> falling */
        ESPIMODE_3,    /*!< sck idle -> high level, data sample time -> falling */
        ESPIMODE_4     /*!< sck idle -> high level, data sample time -> rising */
    };

    ebus_spimaster() = default;
    virtual ~ebus_spimaster() = default;

    void configparams(enum ESPIMODE _mode)
    {
        _pmode = _mode;
    }
    
    virtual int handle_inout(const void *pout, void *pin, unsigned int count) = 0;
public:
    uint8_t _pmode;
};

} // namespace ESAF

#endif  // ebus_spimaster_H
