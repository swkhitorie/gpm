#include "espeedctrl.h"

Esc::Esc(float min, float max, int sel_pwm, int sel_cnt, ESAF::drv_pwm_generator *pwm, ESAF::drv_pulse_counter *cnter) :
    selec_pwm_channel(sel_pwm),
    selec_pulse_cnt(sel_cnt),
    _pwm_min(min),
    _pwm_max(max),
    taget_speed(0),
    poly_param_1(1),
    poly_param_2(0),
    speed(0.0f),
    speed_param(0),
    _lst_cnt(0),
    _cnt(0),
    _vel_delay_cnt(0),
    _online(false),
    _pwm(pwm),
    _cnter(cnter) {}
Esc::~Esc(){}

void Esc::configpwm(ESAF::drv_pwm_generator *pwm)
{
    _pwm = pwm;
}

void Esc::configcnter(ESAF::drv_pulse_counter *cnter)
{
    _cnter = cnter;
}
    
void Esc::ctrlthrottle(float _percent)
{
    if (_pwm == nullptr) {
        return;
    }
    if (_percent < 0.0f || _percent > 100.0f) {
        _percent = 0.0f;
    }

    _pwm->generate_pulse(selec_pwm_channel, 10 * (_pwm_min + _percent * (_pwm_max - _pwm_min) / 100));
}

void Esc::set_speedpolyparam(float param1, float param2)
{
    poly_param_1 = param1;
    poly_param_2 = param2;
}

void Esc::set_speedradioparam(float param)
{
    speed_param = param;
}

void Esc::updatepolyspeed(float throttle)
{
    taget_speed = poly_param_1 * throttle + poly_param_2;
}

void Esc::updatecnter(float dt)
{
    if (_cnter == nullptr) {
        return;
    }
    
    _cnt = _cnter->get_pulse_count(selec_pulse_cnt);
    
    speed = speed + 0.002f * (speed_param * (_cnt - _lst_cnt) / dt - speed);
    _lst_cnt = _cnt;

    float err = fabs(speed - taget_speed);

    if (err > 300 && err > 0.4f * taget_speed) {
        _vel_delay_cnt++;
    }else {
        _vel_delay_cnt = 1;
        _online = true;
    }
    
    if (_vel_delay_cnt > 1000) {
        _vel_delay_cnt = 1100;
        _online = false;
    }    
}

