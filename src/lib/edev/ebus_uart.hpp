#ifndef __EBUS_UART_H_
#define __EBUS_UART_H_

#include "edev.hpp"
#include "stdint.h"

namespace ESAF
{

class ebus_uart : public EDev
{
public:
    enum EWORDLEN {
        EUART_WORDLEN_7B,
        EUART_WORDLEN_8B,
        EUART_WORDLEN_9B
    };

    enum ESTOPBITS {
        EUART_STOPBITS_0_5,
        EUART_STOPBITS_1,
        EUART_STOPBITS_1_5,
        EUART_STOPBITS_2
    };

    enum EPARITY {
        EUART_PARITY_NONE,
        EUART_PARITY_EVEN,
        EUART_PARITY_ODD
    };

    ebus_uart() = default;
    virtual ~ebus_uart() = default;

    void configparams(uint32_t _baud, enum EWORDLEN _wordlen, enum ESTOPBITS _stopbits, enum EPARITY _parity)
    {
        _pbaud = _baud;
        _pwordlen = _wordlen;
        _pstopbits = _stopbits;
        _pparity = _parity;
    }

public:
    uint32_t _pbaud;
    uint8_t _pwordlen;
    uint8_t _pstopbits;
    uint8_t _pparity;
};

} // namespace ESAF

#endif  // ebus_uart_H
