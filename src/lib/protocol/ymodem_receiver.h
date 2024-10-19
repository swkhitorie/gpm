
#ifndef __YMODEM_RECEIVER_H_
#define __YMODEM_RECEIVER_H_
#include <cmath>
#include <cstring>

#ifndef NULL
#    define NULL (0x00)
#endif
#define YMODEM_SOH (0x01)
#define YMODEM_STX (0x02)
#define YMODEM_EOT (0x04)
#define YMODEM_ACK (0x06)
#define YMODEM_NAK (0x15)
#define YMODEM_CA (0x18)
#define YMODEM_C (0x43)
#define PACKLEN_SOH (133)
#define PACKLEN_STX (1029)
#define MIN_ACK_PTOROCOL_STORE (2)

#ifdef __GNUC__
#    define __YMODEM_WEAK __attribute__((weak))
#else
#    define __YMODEM_WEAK __weak
#endif

extern "C" {
/*! @brief weak function for final ymodem send, default to do nothing
 * @param data single byte to send
 */
void ymodem_write(const char &data);

/*! @brief weak function for ymodem clear write buf, default to do nothing
 */
void ymodem_clear_writebuf(void);

/*! @brief weak function for ymodem clear read buf, default to do nothing
 */
void ymodem_clear_readbuf(void);
}

namespace ESAF
{

enum YMODEM_STATE
{
    PROCESS_START_DATA_TRANS,
    PROCESS_START_SEND_ACK,
    PROCESS_START_SEND_C,
    PROCESS_TRANS_RECEIVE_DATA,
    PROCESS_TRANS_END_SEND_C,
    PROCESS_TRANS_FINAL_ACK,
    PROCESS_TRANS_COMPLETED
};

enum YMODEM_FRAME
{
    FRAME_START_SOH,
    FRAME_DATA_SOH,
    FRAME_DATA_STX,
    FRAME_DATA_EOT,
    FRAME_END_SOH,
    FRAME_NONE
};

/*! @brief ymodem receiver status
 *
 */
struct ymodem_receiver_status
{
    char *_pbuffer;                  /*!< pointer receive data array */
    unsigned int _buffer_len;        /*!< length of receive data array */
    enum YMODEM_STATE _state;        /*!< current state of ymodem receive process, See YMODEM_STATE */
    enum YMODEM_FRAME _frametype;    /*!< current frame type of ymodem receive process YMODEM_FRAME */
    unsigned int _lst_pack_num;      /*!< last pack sequence number */
    unsigned char _eotcount;         /*!< count of eot */
    unsigned int _nfilesize_cal;     /*!< filesize of calculation */
    unsigned int _nfile_size;        /*!< filesize of receive */
    char _file_size[50];             /*!< filesize string */
    char _file_name[50];             /*!< filename string */
};

/*! @brief ymodem receiver - for bootloader
 *
 */
class ymodem_receiver
{
  public:

    /**
     * @brief number string to int
     * @param str pointer to number string
     * @param number output number
     */
    static void stringtoInt(const char *str, unsigned int &number);
    
    /**
     * @brief prescaler running
     * @param prescaler number of multi frequency
     * @return bool true - run, false - wait
     */
    static bool wait_to_run(unsigned char prescaler);

    /**
     * @brief put data to receive buffer
     * @param _status ymodem_receiver_status msg
     * @param pdata pointer to data which will be copied
     * @param len len of data array
     */
    static void put_into_buffer(struct ymodem_receiver_status &_status,
                                const unsigned char *pdata,
                                unsigned int len);
    
    /**
     * @brief CRC16-XMODEM function
     * @param data pointer to check data array
     * @param datalen length of check data array
     * @return unsigned short result of CRC16-XMODEM
     */
    static unsigned short CRC16_XMODEM(const unsigned char *data, unsigned int datalen);

    /**
     * @brief inner ymodem receive state machine
     * @param _status ymodem_receiver_status msg
     * @param pdata receive data array
     * @param size size of receive data array
     * @param retrieve number of data should be retrieve
     * @return char result of receive process
     * 0x00 - success to receive
     * 0x01 - receive array too short or too big
     * 0x02 - too much cumulative data
     * 0x03 - no enough data
     * 0x14 - (SOH) no enough payload data
     * 0x15 - (SOH) sequence number error
     * 0x16 - (SOH) start soh sequence number error
     * 0x17 - (SOH) start soh CRC error or 
     *              final soh data error(bug in if file size % 1024 == 128) or
     *              data soh CRC error 
     * 0x18 - (SOH) start soh filename array error
     * 0x19 - (SOH) start soh filesize array error
     * 0x1F - (SOH) logic error
     * 0x24 - (STX) no enough payload data
     * 0x25 - (STX) sequence number error
     * 0x27 - (STX) stx CRC error
     * 0x2F - (STX) logic error
     * 0xFF - unkown error
     */
    static char recv_statemachine(struct ymodem_receiver_status &_status,
                                  const unsigned char *pdata,
                                  unsigned int size,
                                  unsigned int &retrieve);

    /**
     * @brief outer ymodem receive state machine
     * @param _status ymodem_receiver_status msg
     * @param pdata receive data array
     * @param size size of receive data array
     * @param retrieve number of data should be retrieve
     * @return char result of receive process, same with recv_statemachine()
     */
    static char ymodem_receiver_process(struct ymodem_receiver_status &_status,
                                        const unsigned char *pdata,
                                        unsigned int size,
                                        unsigned int &retrieve);
};

}  // namespace ESAF

#endif  // ymodem_receiver_H
