#ifndef RTCM_HELPER_H_
#define RTCM_HELPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rtkcmn.h"

#define RTCM3_PREAMBLE                     (0xD3)
#define RTCM_INITIAL_BUFFER_LENGTH         (512)

#define RTCM_NONPAYLOAD_LEN                (6)
#define RTCM_PREPAYLOAD_LEN                (3)
#define RTCM_MIN_LENGTH                    (RTCM_NONPAYLOAD_LEN + 10)

typedef enum {
    RTCM_FRAMING_INCOMPLETE = 0,
	RTCM_FRAMING_OK = 1,
	RTCM_FRAMING_BAD_CHECK = 2
} rtcm_framing_t;

typedef enum {
	RTCM_STATE_UNINIT = 0,
	RTCM_STATE_IDLE,
	RTCM_STATE_GOT_PREAMBLE,
	RTCM_STATE_GOT_SYNC,
	RTCM_STATE_GOT_SYNC_LEN,
	RTCM_STATE_GOT_PAYLOAD,
	RTCM_STATE_GOT_CHECKA,
	RTCM_STATE_GOT_CHECKB,
	RTCM_STATE_GOT_BAD_CHECK,
} rtcm_parse_state_t;

typedef struct _ubx_status {
    uint8_t msg_received;
    rtcm_parse_state_t parse_state;
    rtcm_parse_state_t parse_state_lst;
    uint8_t payload_idx;
    uint8_t payload_idx_lst;
    uint16_t cnt_lost;
} rtcm_status_t;

typedef struct _ubx_parse {
    uint8_t preamble;
    uint8_t sync;
    uint8_t sync_len;
    uint16_t payload_len;
    uint8_t cka;
    uint8_t ckb;
    uint8_t ckc;
    uint32_t crc;
    uint8_t payload[RTCM_INITIAL_BUFFER_LENGTH];
} rtcm_msg_t;

uint8_t rtcm_frame_char_buffer(rtcm_msg_t *msg, rtcm_status_t *status, uint8_t c);

#ifdef __cplusplus
}
#endif

#endif

