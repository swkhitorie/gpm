#include "uidlink_protocol.h"
using namespace ESAF;

__UIDPWEAK void uidlink_lowlevel_send_uart(const char *buf, uint16_t len) {}

UIDLink::UIDLink()
    : frame_header(UIDLINK_DEFAULT_FRAME_HEAD),
      frame_tail(UIDLINK_DEFAULT_FRAME_TAIL),
      user_info_num(0)
{
    for (uint8_t i = 0; i < UIDLINK_MAX_USER_FUNCTION_CODE_NUM; i++)
    {
        user_protocol_info[i][0] = 0;
        user_protocol_info[i][1] = 0;
    }
    memset(&_uidlink_rx_msg, 0, sizeof(_uidlink_rx_msg));
    memset(&_uidlink_status, 0, sizeof(_uidlink_status));
}

UIDLink::UIDLink(uint8_t frame_head, uint8_t frame_tail)
    : frame_header(frame_head), frame_tail(frame_tail), user_info_num(0)
{
    for (uint8_t i = 0; i < UIDLINK_MAX_USER_FUNCTION_CODE_NUM; i++)
    {
        user_protocol_info[i][0] = 0;
        user_protocol_info[i][1] = 0;
    }
    memset(&_uidlink_rx_msg, 0, sizeof(_uidlink_rx_msg));
    memset(&_uidlink_status, 0, sizeof(_uidlink_status));
}

void UIDLink::set_user_protocol(uint8_t _function_code, uint8_t _payload_len)
{
    if (user_info_num >= UIDLINK_MAX_USER_FUNCTION_CODE_NUM)
        return;
    user_protocol_info[user_info_num][0] = _function_code;
    user_protocol_info[user_info_num][1] = _payload_len;
    user_info_num++;
}

uidlink_status_t *UIDLink::get_status()
{
    return &(this->_uidlink_status);
}

uidlink_message_t *UIDLink::get_buffer()
{
    return &(this->_uidlink_rx_msg);
}

uint8_t (*UIDLink::get_user_info())[2]
{
    return (this->user_protocol_info);
}

uint8_t UIDLink::get_user_info_num()
{
    return this->user_info_num;
}

void UIDLink::reset_status()
{
    uidlink_status_t *status = get_status();
    status->parse_state      = UIDLINK_PARSE_STATE_IDLE;
}

uint16_t msg_get_send_buffer_length(const uidlink_message_t *msg)
{
    return msg->len + UIDLINK_NUM_NON_PAYLOAD_BYTES;
}

uint16_t UIDLink::finalize_message(uidlink_message_t *msg, uint8_t function_code, uint8_t length)
{
    msg->head                    = frame_header;
    msg->seq                     = get_status()->current_tx_seq;
    get_status()->current_tx_seq = get_status()->current_tx_seq + 1;
    msg->len                     = length;
    msg->function_code           = function_code;
    msg->tail                    = frame_tail;
    for (uint8_t i = 0; i < length; i++)
        update_checksum(msg, msg->payload[i]);

    return length + UIDLINK_NUM_NON_PAYLOAD_BYTES;
}

uint16_t UIDLink::finalize_packet_send(uint8_t msg_funcode, const char *packet, uint8_t length)
{
    uint8_t buf_end[2] = {0, 0};
    uint8_t buf[UIDLINK_NUM_HEADER_BYTES];
    uidlink_status_t *status = get_status();
    buf[0]                   = frame_header;
    buf[1]                   = length;
    buf[2]                   = status->current_tx_seq;
    buf[3]                   = msg_funcode;
    status->current_tx_seq++;

    for (uint8_t i = 0; i < length; i++)
        buf_end[0] += packet[i];
    buf_end[0] += buf[1];
    buf_end[0] += buf[2];
    buf_end[0] += buf[3];

    buf_end[1] = frame_tail;

    _lowlevel_start_uart_send(UIDLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);
    _lowlevel_send_uart((const char *)buf, UIDLINK_NUM_HEADER_BYTES);
    _lowlevel_send_uart(packet, length);
    _lowlevel_send_uart((const char *)buf_end, 2);
    _lowlevel_end_uart_send(UIDLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)length);

    return (UIDLINK_NUM_NON_PAYLOAD_BYTES + length);
}

uint16_t UIDLink::msg_to_send_buffer(uint8_t *buffer, const uidlink_message_t *msg)
{
    memcpy(buffer, (const uint8_t *)&msg->head, UIDLINK_NUM_HEADER_BYTES + (uint16_t)msg->len);

    uint8_t *ck   = buffer + (UIDLINK_NUM_HEADER_BYTES + (uint16_t)msg->len);
    uint8_t *tail = ck + 1;
    ck[0]         = (uint8_t)(msg->checksum);
    tail[0]       = frame_tail;

    return UIDLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg->len;
}

void UIDLink::resend_uart(uidlink_message_t *msg)
{
    uint8_t checksum = msg->len + msg->seq + msg->function_code;

    for (uint8_t i = 0; i < msg->len; i++)
        update_checksum(msg, msg->payload[i]);

    _lowlevel_start_uart_send(UIDLINK_NUM_NON_PAYLOAD_BYTES + msg->len);
    _lowlevel_send_uart((const char *)(&msg->head), UIDLINK_NUM_HEADER_BYTES);
    _lowlevel_send_uart(_UID_PAYLOAD(msg), msg->len);
    _lowlevel_send_uart((const char *)(&checksum), 1);
    _lowlevel_send_uart((const char *)(&msg->tail), 1);
    _lowlevel_end_uart_send(UIDLINK_NUM_NON_PAYLOAD_BYTES + msg->len);
}

void UIDLink::start_checksum(uidlink_message_t *msg)
{
    msg->checksum = 0x00;
}

void UIDLink::update_checksum(uidlink_message_t *msg, uint8_t c)
{
    msg->checksum += c;
}

uint8_t UIDLink::frame_char(uidlink_message_t *rxmsg,
                            uidlink_status_t *status,
                            uint8_t c,
                            uidlink_message_t *r_message,
                            uidlink_status_t *r_uidlink_status,
                            uint8_t (*user_info)[2],
                            uint8_t user_info_num)
{
    uint8_t i;
    status->msg_received = UIDLINK_FRAMING_INCOMPLETE;

    switch (status->parse_state)
    {
        case UIDLINK_PARSE_STATE_UNINIT:
        case UIDLINK_PARSE_STATE_IDLE:
            if (c == frame_header)
            {
                status->parse_state = UIDLINK_PARSE_STATE_GOT_HEAD;
                rxmsg->len          = 0;
                rxmsg->head         = c;
                start_checksum(rxmsg);
            }
            break;
        case UIDLINK_PARSE_STATE_GOT_HEAD:
            if (status->msg_received
#if (UIDLINK_MAX_PAYLOAD_LEN < 255)
                || c > MAVLINK_MAX_PAYLOAD_LEN
#endif
            )
            {
                status->buffer_overrun++;
                status->parse_error++;
                status->msg_received = UIDLINK_FRAMING_INCOMPLETE;
                status->parse_state  = UIDLINK_PARSE_STATE_IDLE;
            }
            else
            {
                rxmsg->len         = c;
                status->packet_idx = 0;
                update_checksum(rxmsg, c);
                status->parse_state = UIDLINK_PARSE_STATE_GOT_LENGTH;
            }
            break;

        case UIDLINK_PARSE_STATE_GOT_LENGTH:
            rxmsg->seq = c;
            update_checksum(rxmsg, c);
            status->parse_state = UIDLINK_PARSE_STATE_GOT_SEQ;
            break;

        case UIDLINK_PARSE_STATE_GOT_SEQ:
            for (i = 0; i < user_info_num; i++)
                if (user_info[i][0] == c && user_info[i][1] == rxmsg->len)
                    break;

            if (i == user_info_num)
            {
                status->parse_error++;
                status->parse_state = UIDLINK_PARSE_STATE_IDLE;
                break;
            }

            rxmsg->function_code = c;
            update_checksum(rxmsg, c);
            if (rxmsg->len != 0)
            {
                status->parse_state = UIDLINK_PARSE_STATE_GOT_FUNCODE;
            }
            else
            {
                status->parse_state = UIDLINK_PARSE_STATE_GOT_PAYLOAD;
            }
            break;

        case UIDLINK_PARSE_STATE_GOT_FUNCODE:
            _UID_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
            update_checksum(rxmsg, c);
            if (status->packet_idx == rxmsg->len)
            {
                status->parse_state = UIDLINK_PARSE_STATE_GOT_PAYLOAD;
            }
            break;

        case UIDLINK_PARSE_STATE_GOT_PAYLOAD:
            if (c != rxmsg->checksum)
            {
                status->parse_state = UIDLINK_PARSE_STATE_GOT_BAD_CHECKSUM;
            }
            else
            {
                status->parse_state = UIDLINK_PARSE_STATE_GOT_CHECKSUM;
            }
            _UID_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx] = (char)c;
            break;

        case UIDLINK_PARSE_STATE_GOT_CHECKSUM:
        case UIDLINK_PARSE_STATE_GOT_BAD_CHECKSUM:
            if (status->parse_state == UIDLINK_PARSE_STATE_GOT_BAD_CHECKSUM || c != frame_tail)
            {
                status->msg_received = UIDLINK_FRAMING_BAD_CHECKSUM;
            }
            else
            {
                status->msg_received = UIDLINK_FRAMING_OK;
            }
            _UID_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx + 1] = (char)c;
            memcpy((uint8_t *)r_message, (uint8_t *)rxmsg, sizeof(uidlink_message_t));
            break;
    }

    /* If a message has been sucessfully decoded, check index */
    if (status->msg_received == UIDLINK_FRAMING_OK)
    {
        // while(status->current_seq != rxmsg->seq)
        //{
        //	status->packet_rx_drop_count++;
        //    status->current_seq++;
        //}
        status->current_rx_seq = rxmsg->seq;
        /* Initial condition: If no packet has been received so far, drop count is
         * undefined */
        if (status->packet_rx_success_count == 0)
            status->packet_rx_drop_count = 0;
        /* Count this packet as received */
        status->packet_rx_success_count++;
    }

    r_message->len                            = rxmsg->len;
    r_uidlink_status->parse_state             = status->parse_state;
    r_uidlink_status->packet_idx              = status->packet_idx;
    r_uidlink_status->current_rx_seq          = status->current_rx_seq + 1;
    r_uidlink_status->packet_rx_success_count = status->packet_rx_success_count;
    r_uidlink_status->packet_rx_drop_count    = status->parse_error;
    status->parse_error                       = 0;

    if (status->msg_received == UIDLINK_FRAMING_BAD_CHECKSUM)
    {
        r_message->checksum = _UID_PAYLOAD(rxmsg)[status->packet_idx];
    }

    if (status->msg_received == UIDLINK_FRAMING_BAD_CHECKSUM)
        return frame_tail;
    else
        return status->msg_received;
}

uint8_t UIDLink::parse_char(uint8_t c,
                            uidlink_message_t *r_message,
                            uidlink_status_t *r_uidlink_status)
{
    return frame_char(get_buffer(), get_status(), c, r_message, r_uidlink_status, get_user_info(),
                      get_user_info_num());
}
