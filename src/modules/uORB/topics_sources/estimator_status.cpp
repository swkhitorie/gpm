/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file estimator_status.msg */


#include <inttypes.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/uORBTopics.hpp>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>

constexpr char __orb_estimator_status_fields[] = "uint64_t timestamp;float[24] states;float[3] vibe;float[24] covariances;float[3] output_tracking_error;uint32_t control_mode_flags;float pos_horiz_accuracy;float pos_vert_accuracy;float mag_test_ratio;float vel_test_ratio;float pos_test_ratio;float hgt_test_ratio;float tas_test_ratio;float hagl_test_ratio;float beta_test_ratio;float time_slip;uint16_t gps_check_fail_flags;uint16_t filter_fault_flags;uint16_t innovation_check_flags;uint16_t solution_status_flags;uint8_t n_states;bool pre_flt_fail_innov_heading;bool pre_flt_fail_innov_vel_horiz;bool pre_flt_fail_innov_vel_vert;bool pre_flt_fail_innov_height;bool pre_flt_fail_mag_field_disturbed;uint8_t health_flags;uint8_t timeout_flags;uint8_t[4] _padding0;";

ORB_DEFINE(estimator_status, struct estimator_status_s, 284, __orb_estimator_status_fields, static_cast<uint8_t>(ORB_ID::estimator_status));


void print_message(const estimator_status_s &message)
{

	(void)message;
	PX4_INFO_RAW("Not implemented on flash constrained hardware\n");

}