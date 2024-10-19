
#ifndef __SBUS_PARSER_H_
#define __SBUS_PARSER_H_
#include <stdint.h>
#define SBUS_VALUE_MIN (352)
#define SBUS_VALUE_MAX (1632)

namespace ESAF
{

/*! @brief interface class for sbus parse
 *
 */
class sbus_parser
{
  public:
    /*! @brief turn bytes to double value
     * @param frameinput original bytestream frame, 25 bytes
     * @param frameoutput sbus channel value, 16 channels
     * @param online online flag
     * @param cnt_lost lost frame count
     * @param cnt_update normal frame count
     */
    static void sbus_channel_parser(const uint8_t *frameinput,
                                    uint16_t *frameoutput,
                                    uint8_t &online,
                                    uint32_t &cnt_lost,
                                    uint32_t &cnt_update)
    {
        frameoutput[0] = frameinput[1] | ((frameinput[2] & 0x07) << 8);
        frameoutput[1] = (frameinput[2] >> 3) | ((frameinput[3] & 0x3F) << 5);
        frameoutput[2] =
            (frameinput[3] >> 6) | (frameinput[4] << 2) | ((frameinput[5] & 0x01) << 10);
        frameoutput[3] = (frameinput[5] >> 1) | ((frameinput[6] & 0x0F) << 7);
        frameoutput[4] = (frameinput[6] >> 4) | ((frameinput[7] & 0x7F) << 4);
        frameoutput[5] =
            (frameinput[7] >> 7) | (frameinput[8] << 1) | ((frameinput[9] & 0x03) << 9);
        frameoutput[6] = (frameinput[9] >> 2) | ((frameinput[10] & 0x1F) << 6);
        frameoutput[7] = (frameinput[10] >> 5) | (frameinput[11] << 3);
        frameoutput[8] = frameinput[12] | ((frameinput[13] & 0x07) << 8);
        frameoutput[9] = (frameinput[13] >> 3) | ((frameinput[14] & 0x3F) << 5);
        frameoutput[10] =
            (frameinput[14] >> 6) | (frameinput[15] << 2) | ((frameinput[16] & 0x01) << 10);
        frameoutput[11] = (frameinput[16] >> 1) | ((frameinput[17] & 0x0F) << 7);
        frameoutput[12] = (frameinput[17] >> 4) | ((frameinput[18] & 0x7F) << 4);
        frameoutput[13] =
            (frameinput[18] >> 7) | (frameinput[19] << 1) | ((frameinput[20] & 0x03) << 9);
        frameoutput[14] = (frameinput[20] >> 2) | ((frameinput[21] & 0x1F) << 6);
        frameoutput[15] = (frameinput[21] >> 5) | (frameinput[22] << 3);

        for (int i = 0; i < 16; i++)
        {
            if (frameoutput[i] < SBUS_VALUE_MIN)
                frameoutput[i] = SBUS_VALUE_MIN;
            else if (frameoutput[i] > SBUS_VALUE_MAX)
                frameoutput[i] = SBUS_VALUE_MAX;
            frameoutput[i] = (frameoutput[i] - SBUS_VALUE_MIN) / 12.8;
        }

        if (frameinput[23] & 0x08)
            online = 0;
        else
            online = 1;

        if (frameinput[23] & 0x04)
            cnt_lost++;
        else
            cnt_update++;
    }
};

}  // namespace ESAF

#endif  // sbus_paser_H
