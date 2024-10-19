/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/cwkj/PX4-Autopilot/msg/ulog_stream.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define ULOG_STREAM_FLAGS_NEED_ACK 1
#define ULOG_STREAM_ORB_QUEUE_LENGTH 16

#endif


#ifdef __cplusplus
struct __EXPORT ulog_stream_s {
#else
struct ulog_stream_s {
#endif
	uint64_t timestamp;
	uint16_t msg_sequence;
	uint8_t length;
	uint8_t first_message_offset;
	uint8_t flags;
	uint8_t data[249];
	uint8_t _padding0[2]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t FLAGS_NEED_ACK = 1;
	static constexpr uint8_t ORB_QUEUE_LENGTH = 16;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(ulog_stream);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const ulog_stream_s& message);
#endif
