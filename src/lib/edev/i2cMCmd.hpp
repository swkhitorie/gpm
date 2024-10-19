
#ifndef __I2C_MASTER_COMMAND_H_
#define __I2C_MASTER_COMMAND_H_

#include "stddef.h"
#include "stdint.h"
#include "drv_i2c_device.hpp"

namespace ESAF
{

/*!
 * @brief command of i2c master node
 */
class i2cMCmd
{
  public:
    enum ReqRegType {
        MEMADD_SIZE_8BIT = 0x01,
        MEMADD_SIZE_16BIT = 0x02
    };
    
    i2cMCmd() : slaveAddr(0), prcv(NULL), len_trans(0), len_rcv(0) {
        for (uint8_t i = 0; i < 5; i++)
            ptrans[i] = 0;
    }
    ~i2cMCmd() {}

    i2cMCmd(const i2cMCmd &cmd)
    {
        slaveAddr = cmd.slaveAddr;
        
        len_trans = cmd.len_trans;
        for (int i = 0; i < len_trans; i++)
            ptrans[i] = cmd.ptrans[i];
        
        prcv      = cmd.prcv;
        len_rcv   = cmd.len_rcv;
        _isTail    = cmd._isTail;
        _pdevice   = cmd._pdevice;
        _reqregtype = cmd._reqregtype;
    }

    i2cMCmd &operator=(const i2cMCmd &cmd)
    {
        slaveAddr = cmd.slaveAddr;
        
        len_trans = cmd.len_trans;
        for (int i = 0; i < len_trans; i++)
            ptrans[i] = cmd.ptrans[i];
        
        prcv      = cmd.prcv;
        len_rcv   = cmd.len_rcv;
        _isTail    = cmd._isTail;
        _pdevice   = cmd._pdevice;
        _reqregtype = cmd._reqregtype;
        return *this;
    }

    /**
     * @brief set command value
     * @param sAddr slave addr in i2c bus
     * @param ptx buffer send by i2c bus master node
     * @param ptx_len send buffer length
     * @param prx buffer receive by i2c bus master node
     * @param prx_len receive buffer length
     */
    void set(uint16_t sAddr, uint8_t *ptx, uint8_t ptx_len, uint8_t *prx, uint8_t prx_len, bool istail, i2cDevice *device, uint8_t regtype)
    {
        slaveAddr = sAddr;
        
        uint8_t max_len = (ptx_len <= 5 ? ptx_len : 5);
        len_trans = max_len;
        for (int i = 0; i < len_trans; i++)
            ptrans[i] = ptx[i];
        
        prcv      = prx;
        len_rcv   = prx_len;
        _isTail    = istail;
        _pdevice   = device;
        _reqregtype = regtype;
    }

  public:
    uint16_t slaveAddr;
    uint8_t ptrans[5];
    uint8_t *prcv;
    uint8_t len_trans;
    uint8_t len_rcv;
    bool _isTail;
    i2cDevice *_pdevice;
    uint8_t _reqregtype;
};

}  // namespace ESAF

#endif  // i2cMasterCommand_H
