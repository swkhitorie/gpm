
#ifndef __ESPEEDCTRL_H_
#define __ESPEEDCTRL_H_

#include "drv_pwm_generator.hpp"
#include "drv_pulse_counter.hpp"
#include <cstdint>
#include <cmath>

class Esc
{
public:
    Esc() = delete;
    Esc(float min, float max, int sel_pwm = 1, int sel_cnt = 0, ESAF::drv_pwm_generator *pwm = nullptr, ESAF::drv_pulse_counter *cnter = nullptr);
    ~Esc();

    void configpwm(ESAF::drv_pwm_generator *pwm);
    void configcnter(ESAF::drv_pulse_counter *cnter);

    void ctrlthrottle(float _percent);

    void set_speedpolyparam(float param1, float param2);
    void set_speedradioparam(float param);

    void updatepolyspeed(float throttle);
    void updatecnter(float dt);

public:
    int selec_pwm_channel;
    int selec_pulse_cnt;
    float _pwm_min;
    float _pwm_max;

    float speed;
    float taget_speed;
    float poly_param_1;
    float poly_param_2;
    float speed_param;

    int64_t _lst_cnt;
    int64_t _cnt;
    int _vel_delay_cnt;
    bool _online;

    ESAF::drv_pwm_generator *_pwm;
    ESAF::drv_pulse_counter *_cnter;
};


#endif
