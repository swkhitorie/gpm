
#ifndef __SPI_MASTER_COMMAND_H_
#define __SPI_MASTER_COMMAND_H_

#include "stddef.h"
#include "stdint.h"

namespace ESAF
{

/*!
 * @brief command of spi master node
 */
class spiMCmd
{
  public:
    spiMCmd() : ptrans(NULL), prcv(NULL), len(0) {}

    ~spiMCmd() {}

    spiMCmd(const spiMCmd &cmd)
    {
        ptrans = cmd.ptrans;
        prcv   = cmd.prcv;
        len    = cmd.len;
    }

    spiMCmd &operator=(const spiMCmd &cmd)
    {
        ptrans = cmd.ptrans;
        prcv   = cmd.prcv;
        len    = cmd.len;
        return *this;
    }

    /**
     * @brief set command value
     * @param ptx buffer send by spi bus master node
     * @param prx buffer receive by spi bus master node
     * @param len transmit and receive length
     */
    void set(uint8_t *ptx, uint8_t *prx, uint8_t _len)
    {
        ptrans = ptx;
        prcv   = prx;
        len    = _len;
    }

  public:
    uint8_t *ptrans;
    uint8_t *prcv;
    uint8_t len;
};

}  // namespace ESAF

#endif  // spiMasterCommand_H
