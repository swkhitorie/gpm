#include "rtcm_helper.h"

uint8_t rtcm_frame_char_buffer(rtcm_msg_t *msg, rtcm_status_t *status, uint8_t c)
{
	uint32_t tmp_crc = 0;
	status->msg_received = RTCM_FRAMING_INCOMPLETE;
    status->parse_state_lst = status->parse_state;
    status->payload_idx_lst = status->payload_idx;

	switch (status->parse_state) {
	case RTCM_STATE_UNINIT:
	case RTCM_STATE_IDLE:
		if (c == RTCM3_PREAMBLE) {
			msg->payload_len = 0;
			msg->preamble = c;
			rtk_crc24q_init(&msg->crc);
			rtk_crc24q_update(c, &msg->crc);
			status->parse_state = RTCM_STATE_GOT_PREAMBLE;
		}
		break;
	case RTCM_STATE_GOT_PREAMBLE:
		if ((c & 0xfc) == 0x00) {
            msg->sync = c;
            rtk_crc24q_update(c, &msg->crc);
			status->parse_state = RTCM_STATE_GOT_SYNC;
		} else {
            status->parse_state = RTCM_STATE_IDLE;
        }
		break;
	case RTCM_STATE_GOT_SYNC:
		msg->sync_len = c;
		msg->payload_len = ((msg->sync & 0x03) << 8) | msg->sync_len;
		status->payload_idx = 0;
		rtk_crc24q_update(c, &msg->crc);
		status->parse_state = RTCM_STATE_GOT_SYNC_LEN;
		break;
	case RTCM_STATE_GOT_SYNC_LEN:
		msg->payload[status->payload_idx++] = c;
		rtk_crc24q_update(c, &msg->crc);
		if (status->payload_idx == msg->payload_len) {
			status->parse_state = RTCM_STATE_GOT_PAYLOAD;
		}
		break;
	case RTCM_STATE_GOT_PAYLOAD:
		msg->cka = c;
		status->parse_state = RTCM_STATE_GOT_CHECKA;
		break;
	case RTCM_STATE_GOT_CHECKA:
		msg->ckb = c;
		status->parse_state = RTCM_STATE_GOT_CHECKB;
		break;
	case RTCM_STATE_GOT_CHECKB:
	case RTCM_STATE_GOT_BAD_CHECK:
		msg->ckc = c;
		tmp_crc = ((uint32_t)msg->cka << 16 | (uint32_t)msg->ckb << 8 | (uint32_t)msg->ckc);
		if (msg->crc != tmp_crc) {
			status->msg_received = RTCM_FRAMING_BAD_CHECK;
		} else {
			status->msg_received = RTCM_FRAMING_OK;
		}
		status->parse_state = RTCM_STATE_IDLE;
		break;
	}

	return status->msg_received;
}
