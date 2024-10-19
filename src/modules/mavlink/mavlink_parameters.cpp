#include "mavlink_vehicle.h"

static bool mavstrcmp(const char *p1, const char *p2)
{
    for (int i = 0; i < 16; i++) {
        if (p1[i] != p2[i])
            return true;
    }
    return false;
}

void MavlinkVehicle::handle_msg_param_set(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_param_set_t pset_t;    
    mavlink_msg_param_set_decode(&_rrxmsg, &pset_t);

    int i = 0;
    UParams::param_value_t *p = UParams::get_head();
    int psize = UParams::param_list_size();
    for (; i < psize; i++) {
        for (int j = 0; j < 16; j++)
        if (!mavstrcmp(p[i].param_id, pset_t.param_id)) {
            p[i].param_value = pset_t.param_value;
        }
    }
    
    mavlink_param_value_t back_param;
    strcpy(back_param.param_id, pset_t.param_id);
    back_param.param_value = pset_t.param_value;
    back_param.param_index = 0;
    back_param.param_count = 1;
    mavlink_msg_param_value_send_struct(MAVLINK_COMM_0, &back_param);
}

void MavlinkVehicle::handle_msg_param_request_list(mavlink_message_t *msg)
{
    _now_dwparams.active = true;
    _now_dwparams.index_inlist = _now_dwparams.interval_waitcnt = 0;
    _now_dwparams.step = _now_dwparams.lst_step = 0;
    GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "Reading Board Paramlist");
}

void MavlinkVehicle::handle_msg_param_request_read(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_param_request_read_t pread_t;    
    mavlink_msg_param_request_read_decode(&_rrxmsg, &pread_t);
    
    int i = 0;
    mavlink_param_value_t tmp_mparam;
    UParams::param_value_t *p = UParams::get_head();
    int psize = UParams::param_list_size();
    
    for (; i < psize; i++) {
        for (int j = 0; j < 16; j++)
        if (!mavstrcmp(p[i].param_id, pread_t.param_id)) {
            handle_msg_uparams_to_mavparams(&p[i], &tmp_mparam);
            tmp_mparam.param_count = 1;
            tmp_mparam.param_index = 0;
            mavlink_msg_param_value_send_struct(MAVLINK_COMM_0, &tmp_mparam);
            return;
        }
    }
    
    if (i == psize) {
        GCS_SEND_TEXT(chan, MAV_SEVERITY_INFO, "failed id: %s", pread_t.param_id);
    }
}

bool MavlinkVehicle::handle_msg_param_download()
{
    if (_now_dwparams.active) {
        mavlink_param_value_t tmp_mparam;
        UParams::param_value_t *p = UParams::get_head();
        
        if (_now_dwparams.index_inlist >= UParams::param_list_size()) {
            _now_dwparams.active = false;
            _now_dwparams.index_inlist = _now_dwparams.interval_waitcnt = 0;
            _now_dwparams.step = _now_dwparams.lst_step = 0;
            return _now_dwparams.active;
            
        }else {
            if (UParams::param_list_size() < 300) {
                for (int i = 0; i < UParams::param_list_size(); i++) {
                    handle_msg_uparams_to_mavparams(&p[_now_dwparams.index_inlist], &tmp_mparam);
                    tmp_mparam.param_count = UParams::param_list_size();
                    tmp_mparam.param_index = _now_dwparams.index_inlist;
                    mavlink_msg_param_value_send_struct(chan, &tmp_mparam);
                    _now_dwparams.index_inlist++; 
                }   
            }else {
                if (_now_dwparams.step <= 10) {
                    handle_msg_uparams_to_mavparams(&p[_now_dwparams.index_inlist], &tmp_mparam);
                    tmp_mparam.param_count = UParams::param_list_size();
                    tmp_mparam.param_index = _now_dwparams.index_inlist;
                    mavlink_msg_param_value_send_struct(chan, &tmp_mparam);
                    _now_dwparams.index_inlist++;
                    if (_now_dwparams.index_inlist > (_now_dwparams.step + 1) * UParams::param_list_size() / 10) {
                        _now_dwparams.lst_step = _now_dwparams.step;
                        _now_dwparams.step = 0xff;
                    }
                    
                } else if (_now_dwparams.step == 0xff) {
                    _now_dwparams.interval_waitcnt++;
                    //debug cnt : 400
                    if (_now_dwparams.interval_waitcnt > 50) {
                        _now_dwparams.interval_waitcnt = 0;
                        _now_dwparams.step = _now_dwparams.lst_step + 1;
                    }
                }
            }
        }
    }
    return _now_dwparams.active;
}

bool MavlinkVehicle::handle_msg_param_download_m2()
{
    if (_now_dwparams.active) {
        mavlink_param_value_t tmp_mparam;
        UParams::param_value_t *p = UParams::get_head();
        
        if (_now_dwparams.index_inlist >= UParams::param_list_size()) {
            _now_dwparams.active = false;
            _now_dwparams.index_inlist = _now_dwparams.interval_waitcnt = 0;
            _now_dwparams.step = _now_dwparams.lst_step = 0;
            return _now_dwparams.active;
            
        } else {
            if (++_now_dwparams.interval_waitcnt > 3) {
                _now_dwparams.interval_waitcnt = 0;
                handle_msg_uparams_to_mavparams(&p[_now_dwparams.index_inlist], &tmp_mparam);
                tmp_mparam.param_count = UParams::param_list_size();
                tmp_mparam.param_index = _now_dwparams.index_inlist;
                mavlink_msg_param_value_send_struct(chan, &tmp_mparam);
                _now_dwparams.index_inlist++;  
            }
        }
    }
    return _now_dwparams.active;
}


void MavlinkVehicle::handle_msg_uparams_to_mavparams(UParams::param_value_t* src, mavlink_param_value_t* dst)
{
    switch (src->param_type) {
    case UParams::PARAM_TYPE_INT32: dst->param_type = MAV_PARAM_TYPE_INT32; break;
    case UParams::PARAM_TYPE_FLOAT: dst->param_type = MAV_PARAM_TYPE_REAL32; break;
    case UParams::PARAM_TYPE_BOOL: dst->param_type = MAV_PARAM_TYPE_INT32; break;        
    }
    dst->param_value = src->param_value;
    for (int i = 0; i < 16; i++)
        dst->param_id[i] = src->param_id[i];
}



