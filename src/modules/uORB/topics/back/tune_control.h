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

/* Auto-generated by genmsg_cpp from file /home/cwkj/PX4-Autopilot/msg/tune_control.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define TUNE_CONTROL_TUNE_ID_STOP 0
#define TUNE_CONTROL_TUNE_ID_STARTUP 1
#define TUNE_CONTROL_TUNE_ID_ERROR 2
#define TUNE_CONTROL_TUNE_ID_NOTIFY_POSITIVE 3
#define TUNE_CONTROL_TUNE_ID_NOTIFY_NEUTRAL 4
#define TUNE_CONTROL_TUNE_ID_NOTIFY_NEGATIVE 5
#define TUNE_CONTROL_TUNE_ID_ARMING_WARNING 6
#define TUNE_CONTROL_TUNE_ID_BATTERY_WARNING_SLOW 7
#define TUNE_CONTROL_TUNE_ID_BATTERY_WARNING_FAST 8
#define TUNE_CONTROL_TUNE_ID_GPS_WARNING 9
#define TUNE_CONTROL_TUNE_ID_ARMING_FAILURE 10
#define TUNE_CONTROL_TUNE_ID_PARACHUTE_RELEASE 11
#define TUNE_CONTROL_TUNE_ID_SINGLE_BEEP 12
#define TUNE_CONTROL_TUNE_ID_HOME_SET 13
#define TUNE_CONTROL_TUNE_ID_SD_INIT 14
#define TUNE_CONTROL_TUNE_ID_SD_ERROR 15
#define TUNE_CONTROL_TUNE_ID_PROG_PX4IO 16
#define TUNE_CONTROL_TUNE_ID_PROG_PX4IO_OK 17
#define TUNE_CONTROL_TUNE_ID_PROG_PX4IO_ERR 18
#define TUNE_CONTROL_TUNE_ID_POWER_OFF 19
#define TUNE_CONTROL_NUMBER_OF_TUNES 20
#define TUNE_CONTROL_VOLUME_LEVEL_MIN 0
#define TUNE_CONTROL_VOLUME_LEVEL_DEFAULT 20
#define TUNE_CONTROL_VOLUME_LEVEL_MAX 100
#define TUNE_CONTROL_ORB_QUEUE_LENGTH 4

#endif


#ifdef __cplusplus
struct __EXPORT tune_control_s {
#else
struct tune_control_s {
#endif
	uint64_t timestamp;
	uint32_t duration;
	uint32_t silence;
	uint16_t frequency;
	uint8_t tune_id;
	bool tune_override;
	uint8_t volume;
	uint8_t _padding0[3]; // required for logger


#ifdef __cplusplus
	static constexpr uint8_t TUNE_ID_STOP = 0;
	static constexpr uint8_t TUNE_ID_STARTUP = 1;
	static constexpr uint8_t TUNE_ID_ERROR = 2;
	static constexpr uint8_t TUNE_ID_NOTIFY_POSITIVE = 3;
	static constexpr uint8_t TUNE_ID_NOTIFY_NEUTRAL = 4;
	static constexpr uint8_t TUNE_ID_NOTIFY_NEGATIVE = 5;
	static constexpr uint8_t TUNE_ID_ARMING_WARNING = 6;
	static constexpr uint8_t TUNE_ID_BATTERY_WARNING_SLOW = 7;
	static constexpr uint8_t TUNE_ID_BATTERY_WARNING_FAST = 8;
	static constexpr uint8_t TUNE_ID_GPS_WARNING = 9;
	static constexpr uint8_t TUNE_ID_ARMING_FAILURE = 10;
	static constexpr uint8_t TUNE_ID_PARACHUTE_RELEASE = 11;
	static constexpr uint8_t TUNE_ID_SINGLE_BEEP = 12;
	static constexpr uint8_t TUNE_ID_HOME_SET = 13;
	static constexpr uint8_t TUNE_ID_SD_INIT = 14;
	static constexpr uint8_t TUNE_ID_SD_ERROR = 15;
	static constexpr uint8_t TUNE_ID_PROG_PX4IO = 16;
	static constexpr uint8_t TUNE_ID_PROG_PX4IO_OK = 17;
	static constexpr uint8_t TUNE_ID_PROG_PX4IO_ERR = 18;
	static constexpr uint8_t TUNE_ID_POWER_OFF = 19;
	static constexpr uint8_t NUMBER_OF_TUNES = 20;
	static constexpr uint8_t VOLUME_LEVEL_MIN = 0;
	static constexpr uint8_t VOLUME_LEVEL_DEFAULT = 20;
	static constexpr uint8_t VOLUME_LEVEL_MAX = 100;
	static constexpr uint8_t ORB_QUEUE_LENGTH = 4;

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(tune_control);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const tune_control_s& message);
#endif
