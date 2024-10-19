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

/* Auto-generated by genmsg_cpp from file /home/cwkj/PX4-Autopilot/msg/px4io_status.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus

#endif


#ifdef __cplusplus
struct __EXPORT px4io_status_s {
#else
struct px4io_status_s {
#endif
	uint64_t timestamp;
	float voltage_v;
	float rssi_v;
	uint16_t free_memory_bytes;
	uint16_t pwm[8];
	uint16_t pwm_disarmed[8];
	uint16_t pwm_failsafe[8];
	uint16_t pwm_rate_hz[8];
	uint16_t raw_inputs[18];
	bool status_arm_sync;
	bool status_failsafe;
	bool status_fmu_initialized;
	bool status_fmu_ok;
	bool status_init_ok;
	bool status_outputs_armed;
	bool status_raw_pwm;
	bool status_rc_ok;
	bool status_rc_dsm;
	bool status_rc_ppm;
	bool status_rc_sbus;
	bool status_rc_st24;
	bool status_rc_sumd;
	bool status_safety_off;
	bool alarm_pwm_error;
	bool alarm_rc_lost;
	bool arming_failsafe_custom;
	bool arming_fmu_armed;
	bool arming_fmu_prearmed;
	bool arming_force_failsafe;
	bool arming_io_arm_ok;
	bool arming_lockdown;
	bool arming_termination_failsafe;
	uint8_t _padding0[3]; // required for logger


#ifdef __cplusplus

#endif
};

/* register this as object request broker structure */
ORB_DECLARE(px4io_status);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const px4io_status_s& message);
#endif
