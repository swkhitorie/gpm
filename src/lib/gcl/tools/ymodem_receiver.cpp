
#include "ymodem_receiver.h"
using namespace ESAF;

extern "C" {
__YMODEM_WEAK void ymodem_write(const char &data) {}
__YMODEM_WEAK void ymodem_clear_writebuf() {}
__YMODEM_WEAK void ymodem_clear_readbuf() {}
}

bool ymodem_receiver::wait_to_run(unsigned char prescaler)
{
    static unsigned int _wait_to_run = 0;
    if (_wait_to_run > prescaler)
    {
        _wait_to_run = 1;
        return true;
    }
    else
    {
        _wait_to_run++;
        return false;
    }
}

void ymodem_receiver::put_into_buffer(struct ymodem_receiver_status &_status,
                                      const unsigned char *pdata,
                                      unsigned int len)
{
    memcpy((char *)_status._pbuffer, pdata, len);
    _status._pbuffer += len;
}

unsigned short ymodem_receiver::CRC16_XMODEM(const unsigned char *data, unsigned int datalen)
{
    unsigned short wCRCin = 0x0000;
    unsigned short wCPoly = 0x1021;

    while (datalen--)
    {
        wCRCin ^= (*(data++) << 8);
        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000)
                wCRCin = (wCRCin << 1) ^ wCPoly;
            else
                wCRCin = wCRCin << 1;
        }
    }
    return (wCRCin);
}

void ymodem_receiver::stringtoInt(const char *str, unsigned int &number)
{
    int len = strlen(str);
    int i   = 0;
    for (i = (len - 1); i >= 0; i--)
    {
        number += (str[i] - '0') * powf(10, len - i - 1);
    }
}

char ymodem_receiver::recv_statemachine(struct ymodem_receiver_status &_status,
                                        const unsigned char *pdata,
                                        unsigned int size,
                                        unsigned int &retrieve)
{
    unsigned int index = 0;
    unsigned int i     = 0;
    unsigned int preframe_size;
    unsigned int begin_index = 0;
    unsigned int sub_index   = 0;
    const unsigned char *pframe;

    unsigned char _frame_serialnum     = 0;
    unsigned char _frame_serialnum_xor = 0;
    unsigned int count_fileName        = 0;
    unsigned int count_fileSize        = 0;

    bool last                 = false;
    unsigned int last_datalen = 0;

    if (size < PACKLEN_SOH && size > MIN_ACK_PTOROCOL_STORE)
    {
        retrieve = 0;
        return (char)(0x01);
    }

    if (size > PACKLEN_STX * 2)
    {
        retrieve = size;
        return (char)(0x02);
    }

    for (; index < size; index++)
    {
        if (pdata[index] == YMODEM_SOH || pdata[index] == YMODEM_STX || pdata[index] == YMODEM_EOT)
        {
            begin_index = index;
            pframe      = &pdata[index];
            break;
        }
    }

    if (index == size)
    {
        retrieve = size;
        return (char)(0x03);
    }

    preframe_size = begin_index;
    sub_index     = 0;
    switch (pframe[sub_index])
    {
        case YMODEM_SOH:
        {
            if ((size - begin_index + 1) < PACKLEN_SOH)
            {
                retrieve = preframe_size;
                return (char)(0x14);
            }
            sub_index++;
            _frame_serialnum     = pframe[sub_index++];
            _frame_serialnum_xor = pframe[sub_index++];

            if (_frame_serialnum != 0x00)
            {
                // ====================== check basical serial num is correct or not
                if (_frame_serialnum != (_status._lst_pack_num + 1) ||
                    ((_frame_serialnum + _frame_serialnum_xor) != 0xFF))
                {
                    retrieve = preframe_size + 3;
                    return (char)(0x15);
                }
            }
            // ====================== check start / end soh process ======================
            if (_frame_serialnum == 0x00 && _frame_serialnum_xor == 0xFF)
            {
                if (_status._state != PROCESS_START_DATA_TRANS &&
                    _status._state != PROCESS_TRANS_FINAL_ACK)
                {
                    // ====================== check start end soh error #1
                    retrieve = preframe_size + 3;
                    return (char)(0x16);
                }
                else
                {
                    if (_status._state == PROCESS_START_DATA_TRANS)
                    {
                        // ====================== check start soh process ======================
                        const unsigned char *_data = &pframe[sub_index];
                        if (CRC16_XMODEM(&_data[0], 130) != 0x00)
                        {
                            // ====================== check start soh error #2
                            retrieve = preframe_size + PACKLEN_SOH;
                            return (char)(0x17);
                        }
                        i = 0;
                        while (_data[i] != 0x00)
                        {
                            _status._file_name[count_fileName] = _data[i];
                            i++;
                            if (++count_fileName > 128)
                            {
                                // ====================== check start soh error #3
                                retrieve = preframe_size + PACKLEN_SOH;
                                return (char)(0x18);
                            }
                        }
                        _status._file_name[count_fileName] = '\0';

                        i++;
                        while (_data[i] != 0x00)
                        {
                            _status._file_size[count_fileSize] = _data[i];
                            i++;
                            if (++count_fileSize > 128)
                            {
                                // ====================== check start soh error #4
                                retrieve = preframe_size + PACKLEN_SOH;
                                return (char)(0x19);
                            }
                        }
                        _status._file_size[count_fileSize] = '\0';
                        stringtoInt(&_status._file_size[0], _status._nfile_size);
                        retrieve           = preframe_size + PACKLEN_SOH;
                        _status._frametype = FRAME_START_SOH;
                        return (0x00);
                    }
                    else if (_status._state == PROCESS_TRANS_FINAL_ACK)
                    {
                        // ====================== check end soh process ======================
                        const unsigned char *_data = &pframe[sub_index];
                        for (i = 0; i < 128; i++)
                        {
                            if (_data[i] != 0x00)
                            {
                                // ====================== check end soh error #1
                                retrieve = preframe_size + PACKLEN_SOH;
                                return (char)(0x17);
                            }
                        }
                        retrieve           = preframe_size + PACKLEN_SOH;
                        _status._frametype = FRAME_END_SOH;
                        return (0x00);
                    }
                }
            }

            if (_status._state == PROCESS_TRANS_RECEIVE_DATA)
            {
                // ====================== check data soh process ======================
                _status._lst_pack_num      = _frame_serialnum;
                const unsigned char *_data = &pframe[sub_index];
                if (CRC16_XMODEM(&_data[0], 130) != 0x00)
                {
                    retrieve = preframe_size + PACKLEN_SOH;
                    // ====================== check data soh error #1
                    return (char)(0x17);
                }
				put_into_buffer(_status, &_data[0], 128);
				_status._nfilesize_cal += 128;
                _status._frametype = FRAME_DATA_SOH;
                return (0x00);
            }
            else
            {
                // ====================== logic error #1
                return (char)(0x1F);
            }
            break;
        }
        case YMODEM_STX:
        {
            if ((size - begin_index + 1) < PACKLEN_STX)
            {
                retrieve = preframe_size;
                return (char)(0x24);
            }
            sub_index++;
            _frame_serialnum     = pframe[sub_index++];
            _frame_serialnum_xor = pframe[sub_index++];

            // ====================== check basical serial num is correct or not
            if (_frame_serialnum != (_status._lst_pack_num + 1) ||
                ((_frame_serialnum + _frame_serialnum_xor) != 0xFF))
            {
                retrieve = preframe_size + 3;
                return (char)(0x25);
            }

            if (_status._state == PROCESS_TRANS_RECEIVE_DATA)
            {
                // ====================== check data stx process ======================
                _status._lst_pack_num      = _frame_serialnum;
                const unsigned char *_data = &pframe[sub_index];
                if (CRC16_XMODEM(&_data[0], 1026) != 0x00)
                {
                    retrieve = preframe_size + PACKLEN_SOH;
                    // ====================== check data stx error #1
                    return (char)(0x27);
                }
				
				put_into_buffer(_status, &_data[0], 1024);
				_status._nfilesize_cal += 1024;
                _status._frametype = FRAME_DATA_STX;
                return (0x00);
            }
            else
            {
                // ====================== logic error #2
                return (char)(0x2F);
            }
            break;
        }
        case YMODEM_EOT:
        {
            _status._eotcount++;
            _status._frametype = FRAME_DATA_EOT;
            return 0x00;
            break;
        }
    }
    return 0xFF;
}

char ymodem_receiver::ymodem_receiver_process(struct ymodem_receiver_status &_status,
                                              const unsigned char *pdata,
                                              unsigned int size,
                                              unsigned int &retrieve)
{
    unsigned int pre = 10;
    char res         = 0xff;
    switch (_status._state)
    {
        case PROCESS_START_DATA_TRANS:
        {
            if (!recv_statemachine(_status, pdata, size, retrieve) &&
                _status._frametype == FRAME_START_SOH)
            {
                _status._state = PROCESS_START_SEND_ACK;
                ymodem_clear_writebuf();
            }
            else
            {
                ymodem_write(YMODEM_C);
            }
            break;
        }
        case PROCESS_START_SEND_ACK:
        {
            if (wait_to_run(pre))
            {
                ymodem_write(YMODEM_ACK);
                _status._state = PROCESS_START_SEND_C;
            }
            break;
        }
        case PROCESS_START_SEND_C:
        {
            if (wait_to_run(pre))
            {
                ymodem_write(YMODEM_C);
                _status._state = PROCESS_TRANS_RECEIVE_DATA;
            }
            break;
        }
        case PROCESS_TRANS_RECEIVE_DATA:
        {
            res = recv_statemachine(_status, pdata, size, retrieve);
            if (res == 0 && _status._eotcount == 0 &&
                (_status._frametype == FRAME_DATA_SOH || _status._frametype == FRAME_DATA_STX))
            {
                ymodem_write(YMODEM_ACK);
                ymodem_clear_readbuf();
            }else if (_status._eotcount == 1 && _status._frametype == FRAME_DATA_EOT)
            {
                ymodem_write(YMODEM_NAK);
                ymodem_clear_readbuf();
            }
            else if (_status._eotcount == 2 && _status._frametype == FRAME_DATA_EOT)
            {
                ymodem_write(YMODEM_ACK);
                ymodem_clear_readbuf();
                _status._state = PROCESS_TRANS_END_SEND_C;
            }
            break;
        }
        case PROCESS_TRANS_END_SEND_C:
        {
            if (wait_to_run(pre))
            {
                ymodem_write(YMODEM_C);
                _status._state = PROCESS_TRANS_FINAL_ACK;
            }
            break;
        }
        case PROCESS_TRANS_FINAL_ACK:
        {
            if (!recv_statemachine(_status, pdata, size, retrieve))
            {
                if (_status._frametype == FRAME_END_SOH)
                {
                    ymodem_write(YMODEM_ACK);
                    ymodem_clear_readbuf();
                    _status._state = PROCESS_TRANS_COMPLETED;
                }
                break;
            }
            break;
        }
        case PROCESS_TRANS_COMPLETED:
        {
            break;
        }
    }
    return res;
}
