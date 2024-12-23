
#ifndef __CANDB_MSG_PARSER_H_
#define __CANDB_MSG_PARSER_H_

#include "stdint.h"

namespace ESAF
{
/*!
 * @brief CANDB message parser
 * convert motorola/intel message between signal value
 * only use for little endian memory
 * signal value should be 8/16/32 ...
 */
class candb_msg_parser
{
  public:
    /**
     * @brief get signal value from motorola can message
     * @param data can frame data field
     * @param startbit the offset of the lowest bit of data in the message
     * @param bitlen data len, The position occupied in the message starts with startbit
     *                          and grows towards the high bit of the data
     * @param factor data factor
     * @param offset data offset
     * @return result signal value
     */
    static uint64_t motorola_msg_get_signal(uint8_t *data,
                                            uint8_t startbit,
                                            uint8_t bitlen,
                                            uint32_t factor,
                                            uint32_t offset)
    {
        uint64_t cansignalvalue = 0;
        for (int8_t i = 8 - 1, j = 0; i >= 0; i--, j++)
        {
            cansignalvalue += (uint64_t)data[j] << i * 8;
        }

        int x              = startbit / 8;
        int y              = startbit % 8;
        int z              = (x + 1) * 8 - y;
        int rightMoveCount = 8 * 8 - z;
        cansignalvalue >>= rightMoveCount;
        cansignalvalue = cansignalvalue & (UINT64_MAX >> (64 - bitlen));

        uint64_t values = cansignalvalue * factor + offset;
        return values;
    }

    /**
     * @brief get signal value from intel can message
     * @param data can frame data field
     * @param startbit the offset of the lowest bit of data in the message
     * @param bitlen data len, The position occupied in the message starts with startbit
     *                          and grows towards the high bit of the data
     * @param factor data factor
     * @param offset data offset
     * @return result signal value
     */
    static uint64_t intel_msg_get_signal(uint8_t *data,
                                         uint8_t startbit,
                                         uint8_t bitlen,
                                         uint32_t factor,
                                         uint32_t offset)
    {
        uint64_t cansignalvalue = 0;
        for (int i = 8 - 1; i >= 0; i--)
        {
            cansignalvalue += (uint64_t)data[i] << i * 8;
        }
        int x              = startbit / 8;
        int y              = startbit % 8;
        int rightMoveCount = x * 8 + y;
        cansignalvalue >>= rightMoveCount;
        cansignalvalue  = cansignalvalue & (UINT64_MAX >> (64 - bitlen));
        uint64_t values = cansignalvalue * factor + offset;
        return values;
    }

    /**
     * @brief put signal value in motorola message
     * @param msg motorola message
     * @param values the signal value which will be put
     * @param startbit the offset of the lowest bit of data in the message
     * @param bitlen data len, The position occupied in the message starts with startbit
     *                          and grows towards the high bit of the data
     * @param factor data factor
     * @param offset data offset
     */
    static void signal_to_motorola_msg(uint8_t *msg,
                                       uint64_t values,
                                       uint8_t startbit,
                                       uint8_t bitlen,
                                       uint32_t factor,
                                       uint32_t offset)
    {
        uint64_t data               = uint64_t((values - offset) / factor);
        uint64_t can_signal_motorla = 0;
        uint64_t umask              = 0;
        uint64_t umask_motorla      = 0;
        uint8_t *pdata              = (uint8_t *)&data;
        uint8_t *pmask              = (uint8_t *)&umask;
        uint64_t can_out_signal     = (*(uint64_t *)(&msg[0]));
        uint8_t *p_can_out          = (uint8_t *)&can_out_signal;

        int x          = (startbit / 8) + 1;
        int y          = startbit % 8;
        int move_value = (8 - x) * 8 + y;
        for (uint8_t k = 0; k < bitlen; k++)
        {
            umask |= 1 << k;
        }

        if (bitlen == 32)
        {
            umask &= 0x00000000FFFFFFFF;
        }

        data <<= move_value;
        umask <<= move_value;
        for (int i = (8) - 1, j = 0; i >= 0; i--, j++)
        {
            can_signal_motorla += (uint64_t)pdata[j] << i * 8;
            umask_motorla += (uint64_t)pmask[j] << i * 8;
        }

        can_out_signal &= ~umask_motorla;
        can_out_signal |= can_signal_motorla;

        for (int i = 0; i < 8; i++)
            msg[i] = p_can_out[i];

        return;
    }

    /**
     * @brief put signal value in intel message
     * @param msg motorola message
     * @param values the signal value which will be put
     * @param startbit the offset of the lowest bit of data in the message
     * @param bitlen data len, The position occupied in the message starts with startbit
     *                          and grows towards the high bit of the data
     * @param factor data factor
     * @param offset data offset
     */
    static void signal_to_intel_msg(uint8_t *msg,
                                    uint64_t values,
                                    uint8_t startbit,
                                    uint8_t bitlen,
                                    uint32_t factor,
                                    uint32_t offset)
    {
        uint64_t data             = uint64_t((values - offset) / factor);
        uint64_t can_signal_intel = 0;
        uint64_t umask            = 0;
        uint64_t umask_intel      = 0;
        uint64_t can_out_signal   = (*(uint64_t *)(&msg[0]));
        uint8_t *p_can_out        = (uint8_t *)&can_out_signal;

        for (uint8_t k = 0; k < bitlen; k++)
        {
            umask |= 1 << k;
        }

        data <<= startbit;
        umask <<= startbit;

        can_signal_intel = data;
        umask_intel      = umask;

        can_out_signal &= ~umask_intel;
        can_out_signal |= can_signal_intel;
        for (int i = 0; i < 8; i++)
            msg[i] = p_can_out[i];

        return;
    }
};

}  // namespace ESAF

#endif  // candb_msg_parser_H
