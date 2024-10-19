
#ifndef _UIDLINK_PROTOCOL_H_
#define _UIDLINK_PROTOCOL_H_

#include "uidlink_portable.h"
#include <cstring>

#ifdef __GNUC__
#    define UIDPACKED(__Declaration__) __Declaration__ __attribute__((packed))
#    define __UIDPWEAK __attribute__((weak))
#else
#    define __UIDPWEAK __weak
#    define UIDPACKED(__Declaration__) __packed __Declaration__
#endif
#define UIDLINK_MAX_PAYLOAD_LEN (255)
#define UIDLINK_CORE_HEADER_LEN (3)
#define UIDLINK_NUM_HEADER_BYTES (UIDLINK_CORE_HEADER_LEN + 1)
#define UIDLINK_NUM_CHECKSUM_BYTES (1)
#define UIDLINK_NUM_NON_PAYLOAD_BYTES (UIDLINK_NUM_HEADER_BYTES + UIDLINK_NUM_CHECKSUM_BYTES + 1)

#define UIDLINK_MAX_USER_FUNCTION_CODE_NUM (10)

/* default 'U' '\n' */
#define UIDLINK_DEFAULT_FRAME_HEAD (0x55)
#define UIDLINK_DEFAULT_FRAME_TAIL (0x0A)

namespace ESAF
{

UIDPACKED(typedef struct __uidlink_message {
    uint8_t head;
    uint8_t len;
    uint8_t seq;
    uint8_t function_code;
    uint8_t checksum;
    uint8_t tail;
    uint8_t payload[UIDLINK_MAX_PAYLOAD_LEN];
})
uidlink_message_t;

#define _UID_PAYLOAD(msg) ((const char *)(&((msg)->payload[0])))
#define _UID_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload[0])))

typedef enum
{
    UIDLINK_PARSE_STATE_UNINIT = 0,
    UIDLINK_PARSE_STATE_IDLE,
    UIDLINK_PARSE_STATE_GOT_HEAD,
    UIDLINK_PARSE_STATE_GOT_LENGTH,
    UIDLINK_PARSE_STATE_GOT_SEQ,
    UIDLINK_PARSE_STATE_GOT_FUNCODE,
    UIDLINK_PARSE_STATE_GOT_PAYLOAD,
    UIDLINK_PARSE_STATE_GOT_CHECKSUM,
    UIDLINK_PARSE_STATE_GOT_BAD_CHECKSUM
} uidlink_parse_state_t;

typedef enum
{
    UIDLINK_FRAMING_INCOMPLETE   = 0,
    UIDLINK_FRAMING_OK           = 1,
    UIDLINK_FRAMING_BAD_CHECKSUM = 2
} uidlink_framing_t;

/*! @brief uidlink status structure
 *
 */
typedef struct __uidlink_status
{
    uint8_t msg_received;              /*!< parse process result, See uidlink_framing_t */
    uint8_t buffer_overrun;            /*!< when receive msg payload len greater than 255,
                                          add one */
    uint8_t parse_error;               /*!< when parse process error, add one -->
                                         1 - receive msg not in user frame list
                                         2 - receive msg payload len greater than 255*/
    uidlink_parse_state_t parse_state; /*!< receive parse state, See uidlink_parse_state_t */
    uint8_t packet_idx;                /*!< tmp value of receive payload index */
    uint8_t current_tx_seq;            /*!< serial number of transmit frame */
    uint8_t current_rx_seq;            /*!< serial number of receive frame */
    uint32_t packet_rx_success_count;  /*!< when receive a frame success, add one */
    uint32_t packet_rx_drop_count;     /*!< when receive a frame fail, add one */
} uidlink_status_t;

/*! @brief communication protocol - uidlink
 *  frame structure:
 *  STX LEN SEQ FUNCODE [PAYLOAD] CK END
 */
class UIDLink : public uidlink_lowlevel_layer
{
  public:
    UIDLink();
    UIDLink(uint8_t frame_head, uint8_t frame_tail);

    /**
     * @brief return user frame list
     * @return the user frame list [UIDLINK_MAX_USER_FUNCTION_CODE_NUM][2]
     */
    uint8_t (*get_user_info())[2];

    /**
     * @brief get number of user frame
     * @return uint8_t user frame num
     */
    uint8_t get_user_info_num();

    /**
     * @brief add a frame into user frame list
     * @param _function_code frame function code
     * @param _payload_len frame payload length
     */
    void set_user_protocol(uint8_t _function_code, uint8_t _payload_len);

    /**
     * @brief get uidlink status, no use
     */
    uidlink_status_t *get_status();

    /**
     * @brief get uidlink message, no use
     */
    uidlink_message_t *get_buffer();

    /**
     * @brief reset uidlink status, no use
     */
    void reset_status();

    /**
     * @brief get msg len
     * @return uint16_t bytes of len
     */
    uint16_t msg_get_send_buffer_length(const uidlink_message_t *msg);

    /**
     * @brief fill the message finally
     * @param msg uidlink msg frame
     * @param function_code frame function code
     * @param length frame payload len
     * @return uint16_t bytes of message
     */
    uint16_t finalize_message(uidlink_message_t *msg, uint8_t function_code, uint8_t length);

    /**
     * @brief construct a message and send
     * @param msg_funcode frame function code
     * @param packet frame payload
     * @param length frame payload len
     * @return uint16_t bytes of message
     */
    uint16_t finalize_packet_send(uint8_t msg_funcode, const char *packet, uint8_t length);

    /**
     * @brief convert msg to send bufffer
     * @param buffer output buffer
     * @param msg uidlink msg
     * @return uint16_t bytes of message
     */
    uint16_t msg_to_send_buffer(uint8_t *buffer, const uidlink_message_t *msg);

    /**
     * @brief send a single msg
     * @param msg uidlink msg
     */
    void resend_uart(uidlink_message_t *msg);

    /**
     * @brief start the msg checksum
     * @param msg uidlink msg
     */
    void start_checksum(uidlink_message_t *msg);

    /**
     * @brief update the msg checksum
     * @param msg uidlink msg
     * @param c new data
     */
    void update_checksum(uidlink_message_t *msg, uint8_t c);

    /**
     * @brief parse a frame by single char
     * @param rxmsg inner msg in parse process
     * @param status inner status in parse process
     * @param c new data
     * @param r_message output msg in parse process
     * @param r_mavlink_status output status in parse process
     * @param user_info user frame list
     * @param user_info_num user frame number
     * @return uint8_t msg receive status, See uidlink_framing_t
     */
    uint8_t frame_char(uidlink_message_t *rxmsg,
                       uidlink_status_t *status,
                       uint8_t c,
                       uidlink_message_t *r_message,
                       uidlink_status_t *r_mavlink_status,
                       uint8_t (*user_info)[2],
                       uint8_t user_info_num);

    /**
     * @brief parse a frame by single char, out call
     * @param c new data
     * @param r_message output msg in parse process
     * @param r_mavlink_status output status in parse process
     * @return uint8_t msg receive status, See uidlink_framing_t
     */
    uint8_t parse_char(uint8_t c, uidlink_message_t *r_message, uidlink_status_t *r_uidlink_status);

  public:
    uint8_t frame_header;
    uint8_t frame_tail;

    uint8_t user_info_num;
    uint8_t user_protocol_info[UIDLINK_MAX_USER_FUNCTION_CODE_NUM][2];

    uidlink_message_t _uidlink_rx_msg;
    uidlink_status_t _uidlink_status;
};

}  // namespace ESAF

#endif  // uidlink_protocol_H
