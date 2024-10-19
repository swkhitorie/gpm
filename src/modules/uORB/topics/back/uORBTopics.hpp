/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{206};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	action_request = 0,
	actuator_armed = 1,
	actuator_controls = 2,
	actuator_controls_0 = 3,
	actuator_controls_1 = 4,
	actuator_controls_2 = 5,
	actuator_controls_3 = 6,
	actuator_controls_status = 7,
	actuator_controls_status_0 = 8,
	actuator_controls_status_1 = 9,
	actuator_controls_virtual_fw = 10,
	actuator_controls_virtual_mc = 11,
	actuator_motors = 12,
	actuator_outputs = 13,
	actuator_outputs_sim = 14,
	actuator_servos = 15,
	actuator_servos_trim = 16,
	actuator_test = 17,
	adc_report = 18,
	airspeed = 19,
	airspeed_validated = 20,
	airspeed_wind = 21,
	autotune_attitude_control_status = 22,
	battery_status = 23,
	button_event = 24,
	camera_capture = 25,
	camera_status = 26,
	camera_trigger = 27,
	cellular_status = 28,
	collision_constraints = 29,
	collision_report = 30,
	commander_state = 31,
	control_allocator_status = 32,
	cpuload = 33,
	debug_array = 34,
	debug_key_value = 35,
	debug_value = 36,
	debug_vect = 37,
	differential_pressure = 38,
	distance_sensor = 39,
	ekf2_timestamps = 40,
	esc_report = 41,
	esc_status = 42,
	estimator_attitude = 43,
	estimator_baro_bias = 44,
	estimator_event_flags = 45,
	estimator_global_position = 46,
	estimator_gps_status = 47,
	estimator_innovation_test_ratios = 48,
	estimator_innovation_variances = 49,
	estimator_innovations = 50,
	estimator_local_position = 51,
	estimator_odometry = 52,
	estimator_optical_flow_vel = 53,
	estimator_selector_status = 54,
	estimator_sensor_bias = 55,
	estimator_states = 56,
	estimator_status = 57,
	estimator_status_flags = 58,
	estimator_visual_odometry_aligned = 59,
	estimator_wind = 60,
	event = 61,
	failure_detector_status = 62,
	follow_target = 63,
	fw_virtual_attitude_setpoint = 64,
	generator_status = 65,
	geofence_result = 66,
	gimbal_device_attitude_status = 67,
	gimbal_device_information = 68,
	gimbal_device_set_attitude = 69,
	gimbal_manager_information = 70,
	gimbal_manager_set_attitude = 71,
	gimbal_manager_set_manual_control = 72,
	gimbal_manager_status = 73,
	gimbal_v1_command = 74,
	gps_dump = 75,
	gps_inject_data = 76,
	heater_status = 77,
	home_position = 78,
	hover_thrust_estimate = 79,
	input_rc = 80,
	internal_combustion_engine_status = 81,
	iridiumsbd_status = 82,
	irlock_report = 83,
	landing_gear = 84,
	landing_target_innovations = 85,
	landing_target_pose = 86,
	led_control = 87,
	log_message = 88,
	logger_status = 89,
	mag_worker_data = 90,
	magnetometer_bias_estimate = 91,
	manual_control_input = 92,
	manual_control_setpoint = 93,
	manual_control_switches = 94,
	mavlink_log = 95,
	mavlink_tunnel = 96,
	mc_virtual_attitude_setpoint = 97,
	mission = 98,
	mission_result = 99,
	mount_orientation = 100,
	navigator_mission_item = 101,
	npfg_status = 102,
	obstacle_distance = 103,
	obstacle_distance_fused = 104,
	offboard_control_mode = 105,
	onboard_computer_status = 106,
	optical_flow = 107,
	orbit_status = 108,
	parameter_update = 109,
	ping = 110,
	position_controller_landing_status = 111,
	position_controller_status = 112,
	position_setpoint = 113,
	position_setpoint_triplet = 114,
	power_button_state = 115,
	power_monitor = 116,
	pps_capture = 117,
	pwm_input = 118,
	px4io_status = 119,
	radio_status = 120,
	rate_ctrl_status = 121,
	rc_channels = 122,
	rc_parameter_map = 123,
	rpm = 124,
	rtl_time_estimate = 125,
	safety = 126,
	safety_button = 127,
	satellite_info = 128,
	sensor_accel = 129,
	sensor_accel_fifo = 130,
	sensor_baro = 131,
	sensor_combined = 132,
	sensor_correction = 133,
	sensor_gnss_relative = 134,
	sensor_gps = 135,
	sensor_gyro = 136,
	sensor_gyro_fft = 137,
	sensor_gyro_fifo = 138,
	sensor_hygrometer = 139,
	sensor_mag = 140,
	sensor_preflight_mag = 141,
	sensor_selection = 142,
	sensors_status = 143,
	sensors_status_baro = 144,
	sensors_status_imu = 145,
	sensors_status_mag = 146,
	system_power = 147,
	takeoff_status = 148,
	task_stack_info = 149,
	tecs_status = 150,
	telemetry_status = 151,
	test_motor = 152,
	timesync = 153,
	timesync_status = 154,
	trajectory_bezier = 155,
	trajectory_setpoint = 156,
	trajectory_waypoint = 157,
	transponder_report = 158,
	tune_control = 159,
	uavcan_parameter_request = 160,
	uavcan_parameter_value = 161,
	ulog_stream = 162,
	ulog_stream_ack = 163,
	uwb_distance = 164,
	uwb_grid = 165,
	vehicle_acceleration = 166,
	vehicle_air_data = 167,
	vehicle_angular_acceleration = 168,
	vehicle_angular_acceleration_setpoint = 169,
	vehicle_angular_velocity = 170,
	vehicle_angular_velocity_groundtruth = 171,
	vehicle_attitude = 172,
	vehicle_attitude_groundtruth = 173,
	vehicle_attitude_setpoint = 174,
	vehicle_command = 175,
	vehicle_command_ack = 176,
	vehicle_constraints = 177,
	vehicle_control_mode = 178,
	vehicle_global_position = 179,
	vehicle_global_position_groundtruth = 180,
	vehicle_gps_position = 181,
	vehicle_imu = 182,
	vehicle_imu_status = 183,
	vehicle_land_detected = 184,
	vehicle_local_position = 185,
	vehicle_local_position_groundtruth = 186,
	vehicle_local_position_setpoint = 187,
	vehicle_magnetometer = 188,
	vehicle_mocap_odometry = 189,
	vehicle_odometry = 190,
	vehicle_rates_setpoint = 191,
	vehicle_roi = 192,
	vehicle_status = 193,
	vehicle_status_flags = 194,
	vehicle_thrust_setpoint = 195,
	vehicle_torque_setpoint = 196,
	vehicle_trajectory_bezier = 197,
	vehicle_trajectory_waypoint = 198,
	vehicle_trajectory_waypoint_desired = 199,
	vehicle_vision_attitude = 200,
	vehicle_visual_odometry = 201,
	vtol_vehicle_status = 202,
	wheel_encoders = 203,
	wind = 204,
	yaw_estimator_status = 205,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
