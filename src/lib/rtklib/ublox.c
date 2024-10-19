#include "ublox.h"
#include <stdio.h>
#include <stdlib.h>

void ublox_reset(ubx_status_t *status)
{
    status->msg_received = UBLOX_FRAMING_INCOMPLETE;
    status->parse_state = status->parse_state_lst = UBLOX_STATE_IDLE;
    status->payload_idx = status->payload_idx_lst = 0;
    status->cnt_lost = 0;
}

void ublox_start_checksum(ubx_msg_t* msg, uint8_t c)
{
    msg->cka = 0;
    msg->ckb = 0; 
}

void ublox_update_checksum(ubx_msg_t* msg, uint8_t c)
{
    msg->cka = msg->cka + c;
    msg->ckb = msg->ckb + msg->cka;
}

uint8_t ublox_frame_char_buffer(ubx_msg_t *msg, ubx_status_t *status, uint8_t c)
{
	status->msg_received = UBLOX_FRAMING_INCOMPLETE;
	
        status->parse_state_lst = status->parse_state;
        status->payload_idx_lst = status->payload_idx;
	switch (status->parse_state) {
	case UBLOX_STATE_UNINIT:
	case UBLOX_STATE_IDLE:
		if (c == UBLOX_SYNC_CHAR1) {
			msg->payload_len = 0;
			msg->sync_char1 = c;
			status->parse_state = UBLOX_STATE_GOT_SYNC1;
		}
		break;
	case UBLOX_STATE_GOT_SYNC1:
		if (c == UBLOX_SYNC_CHAR2) {
			msg->sync_char2 = c;
			status->parse_state = UBLOX_STATE_GOT_SYNC2;
		} else {
                  status->parse_state = UBLOX_STATE_IDLE;
                }
		break;
	case UBLOX_STATE_GOT_SYNC2:
		msg->class_id = c;
                ublox_start_checksum(msg, c);
		ublox_update_checksum(msg, c);
		status->parse_state = UBLOX_STATE_GOT_CLASS_ID;
		break;
	case UBLOX_STATE_GOT_CLASS_ID:
		msg->msg_id = c;
		ublox_update_checksum(msg, c);
		status->parse_state = UBLOX_STATE_GOT_MSG_ID;
		break;
	case UBLOX_STATE_GOT_MSG_ID:
		msg->payload_len = c;
		ublox_update_checksum(msg, c);
		status->parse_state = UBLOX_STATE_GOT_PAYLOADLEN_LOW;
		break;
	case UBLOX_STATE_GOT_PAYLOADLEN_LOW:
		msg->payload_len |= c << 8;
		ublox_update_checksum(msg, c);
		status->payload_idx = 0;
		status->parse_state = UBLOX_STATE_GOT_PAYLOADLEN;
		break;
	case UBLOX_STATE_GOT_PAYLOADLEN:
		msg->payload[status->payload_idx++] = c;
		ublox_update_checksum(msg, c);
		if (status->payload_idx == msg->payload_len) {
			status->parse_state = UBLOX_STATE_GOT_PAYLOAD;
		}
		break;
	case UBLOX_STATE_GOT_PAYLOAD:
		msg->ck[0] = c;
		status->parse_state = UBLOX_STATE_GOT_CHECKA;
		break;
	case UBLOX_STATE_GOT_BAD_CHECKA:
	case UBLOX_STATE_GOT_CHECKA:
		msg->ck[1] = c;
		if (msg->cka != msg->ck[0] || msg->ckb != msg->ck[1]) {
			status->msg_received = UBLOX_FRAMING_BAD_CHECK;
		} else {
			status->msg_received = UBLOX_FRAMING_OK;
			
		}
                status->parse_state = UBLOX_STATE_IDLE;
		break;
	}
	
        if (status->parse_state_lst == status->parse_state &&
            status->payload_idx_lst == status->payload_idx) {
              status->cnt_lost++;
              if (status->cnt_lost >= 100) {
                ublox_reset(status);
              }
        } else {
			status->cnt_lost = 0;
        }

	return status->msg_received;
}
