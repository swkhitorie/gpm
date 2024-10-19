#ifndef MAVLINK_VEHICLE_H_
#define MAVLINK_VEHICLE_H_

#include "mavlinkstream.hpp"
#include "mavlinkmsgpack.hpp"
#include "param/parameters.hpp"
#include "sensor_calibration/gyro_calibrator.h"
#include "sensor_calibration/accel_calibrator_bak.h"
#include "sensor_calibration/compass_calibrator.h"

#include "drv_boards.h"

/* 1: SDO(Server-client)-like 2:PDO(Process)-like  */
#define MAVLINK_LOGFILE_TRANSFER_METHOD (2)

#define GCS_SEND_TEXT( _chan, _level, ...) \
        char tmpstring[50];\
        sprintf(&tmpstring[0], __VA_ARGS__);\
        mavlink_msg_statustext_send(_chan, _level, &tmpstring[0], 0, 0);

void mavlink_log_info(mavlink_channel_t chan, char *fmt, ...);
void mavlink_log_critical(mavlink_channel_t chan, char *fmt, ...);
void mavlink_log_notice(mavlink_channel_t chan, char *fmt, ...);
void mavlink_log_debug(mavlink_channel_t chan, char *fmt, ...);
void mavlink_log_warning(mavlink_channel_t chan, char *fmt, ...);
void mavlink_log_error(mavlink_channel_t chan, char *fmt, ...);
void mavlink_log_emergency(mavlink_channel_t chan, char *fmt, ...);

class MavlinkVehicle;
void mavlink_log_erase_all(MavlinkVehicle *obj);
void mavlink_log_request_list(MavlinkVehicle *obj, mavlink_log_request_list_t *prlist_t);
void mavlink_log_request_data(MavlinkVehicle *obj, mavlink_log_request_data_t *prdata_t);
void mavlink_log_data_process(MavlinkVehicle *obj);

class MavlinkVehicle : MavlinkStream
{
public:
    enum TRANSFER_TYPE {
        TRANSFER_IDLE = 0x00,
        GYRO_CALIBRATION,
        ACCEL_CALIBTATION,
        MAG_CALIBRATION,
    };

    typedef struct vuploadmission {
        uint8_t mission_sysid;
        uint8_t mission_compid;
        uint8_t mission_type;
        uint16_t mission_count;
        uint16_t mission_seq;
        bool active;
    } upload_mission_record_t;
    
    typedef struct vdownloadparam {
        uint16_t index_inlist;
        uint16_t interval_waitcnt;
        uint8_t step;
        uint8_t lst_step;
        bool active;
    } download_param_record_t;   
    
    typedef struct logfile {
        uint32_t ofs;
        uint32_t count;
        uint8_t id_entry;
        char path[48];
        bool istransfering;
        bool isfilefound;
    } logfile_transfer_t;
    
    MavlinkVehicle(mavlink_channel_t channel, uint8_t sysid, uint8_t compid);
    ~MavlinkVehicle() = default;

    void broadcast_msg_freqsplit(uint64_t time_boot_ms, uint8_t slice_id, uint16_t period, uint16_t msg_id);
    
    virtual void broadcast(uint64_t time_boot_ms);
    virtual void parsing(uint8_t c);
    void handle_msg_recv(mavlink_message_t *msg);
    
    void handle_msg_heartbeat(mavlink_message_t *msg);
    void handle_msg_ping(mavlink_message_t *msg);
    void handle_msg_timesync(mavlink_message_t *msg);
    
    void handle_msg_set_mode(mavlink_message_t *msg);
    void handle_msg_systime(mavlink_message_t *msg);
    void handle_msg_request_data_stream(mavlink_message_t *msg);
    
    void handle_msg_ftp(mavlink_message_t *msg);
    void handle_msg_log_request_list(mavlink_message_t *msg);
    void handle_msg_log_request_data(mavlink_message_t *msg);
    void handle_msg_log_erase_all(mavlink_message_t *msg);
    
    void handle_msg_uparams_to_mavparams(UParams::param_value_t* src, mavlink_param_value_t* dst);
    void handle_msg_param_request_list(mavlink_message_t *msg);
    void handle_msg_param_request_read(mavlink_message_t *msg);
    void handle_msg_param_set(mavlink_message_t *msg);
    bool handle_msg_param_download();
    bool handle_msg_param_download_m2();
    
    void handle_msg_mission_download_from_vehicle(mavlink_message_t *msg);
    void handle_msg_mission_start_upload_to_vehicle(mavlink_message_t *msg);
    void handle_msg_mission_upload_to_vehicle(mavlink_message_t *msg);
    void handle_msg_mission_get(mavlink_mission_item_int_t *p, uint8_t *size, uint16_t *times);
    
    void handle_msg_command_report();
    void handle_msg_command(mavlink_message_t *msg);
    void handle_msg_command_autopilot_version(mavlink_message_t *msg);
    void handle_msg_command_request_protocol_version(mavlink_message_t *msg);
    void handle_msg_command_set_message_interval(mavlink_message_t *msg);
    void handle_msg_command_request_message(mavlink_message_t *msg);
    void handle_msg_command_preflight_calibration(mavlink_message_t *msg);
    void handle_msg_command_ack(mavlink_message_t *msg);
    void handle_msg_command_mag_cal(mavlink_message_t *msg);
    
    void config_handle_accel_calibration(calibration::AccelCalibratorBak *_acch) { _accel_calibration = _acch; }
    void config_handle_gyro_calibration(calibration::GyroscopeCalibrator *_gyroh) { _gyro_calibration = _gyroh; } 
    void config_handle_compass_calibration(calibration::CompassCalibrator *_copsh) { _compass_calibration = _copsh; } 
    
    void calibration_accel_process();
    void calibration_gyro_process();
    void calibration_compass_process();
    
    MavPacking _msgpak;
    
public:
    uint64_t rtime_boot_ms;
    uint64_t time_utc;
    uint8_t link_sysid;
    uint8_t link_compid;
    uint8_t gcs_sysid;
    uint8_t gcs_compid;
    
    uint64_t ping_rseq;

    /* mavlink transfer period slice */
    uint64_t freq_slice[30];

    /* mavlink command calibration value */
    uint16_t state_machine;
    uint16_t cnt_cal_accel_pos;
    bool start_cal_accel_sample;
    calibration::AccelCalibratorBak *_accel_calibration;
    calibration::GyroscopeCalibrator *_gyro_calibration;
    calibration::CompassCalibrator *_compass_calibration;

    /* mavlink pamameters download/upload detector */
    download_param_record_t _now_dwparams;

    /* mavlink mission download/upload detector */
    upload_mission_record_t _now_upmission;
    mavlink_mission_item_int_t upload_mission[15];
    uint16_t _upload_times;
    
    /* mavlink log file transfer */
    /* min pack size: 90, GCS will auto padding protocol */
    logfile_transfer_t _now_logfile;
    uint8_t log_file_trans_cnt;
};

#endif
