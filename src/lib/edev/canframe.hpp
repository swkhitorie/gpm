
#ifndef __CAN_FRAME_H_
#define __CAN_FRAME_H_

#include "stdint.h"

namespace ESAF
{

/*!
 * @brief frame of standard can
 */
class CANFrame
{
  public:
    enum FrameType
    {
        DATA_FRAME,
        REMOTE_FRAME
    };
    enum IDType
    {
        STANDARD_ID,
        EXTENDED_ID
    };

    CANFrame()
        : _identifier(0),
          _idtype(STANDARD_ID),
          _frametype(DATA_FRAME),
          _datalength(8)
    {
		for (int i = 0; i < 8; i++)
			_data[i] = 0;
	}
    ~CANFrame() {}

    CANFrame(const CANFrame &msg)
    {
        _identifier = msg._identifier;
        _idtype     = msg._idtype;
        _frametype  = msg._frametype;
        _datalength = msg._datalength;
        for (uint8_t i = 0; i < 8; i++)
            _data[i] = msg._data[i];
    }

    CANFrame &operator=(const CANFrame &msg)
    {
        _identifier = msg._identifier;
        _idtype     = msg._idtype;
        _frametype  = msg._frametype;
        _datalength = msg._datalength;
        for (uint8_t i = 0; i < 8; i++)
            _data[i] = msg._data[i];
        return *this;
    }

  public:
    uint32_t _identifier;
    uint32_t _idtype;
    uint32_t _frametype;
    uint32_t _datalength;
    uint8_t _data[8];
};

}  // namespace ESAF

#endif  // canFrame_H
