#include "encoder.h"

Encoder::Encoder(uint32_t ppr, float offset, ESAF::drv_encoder_incremental *coder) :
    _ppr(ppr),
    _zero_angle_offset(offset),
    _zero_count(0),
    _zero_angle(0),
    _raw_count(0),
    _raw_angle(0),
    _count(0),
    _angle(0),
    _lst_angle(0),
    _angle_vel(0),
    _encoder(coder) {}
Encoder::~Encoder() {}

void Encoder::configcoder(ESAF::drv_encoder_incremental *coder)
{
    _encoder = coder;
}


void Encoder::update(float dt)
{
    if (_encoder == nullptr) {
        return;
    }

    _raw_count = _encoder->get_encoder_count();
    _raw_angle = 360.0f * _raw_count / _ppr;

    _count = _raw_count - _zero_count;

    _angle = _raw_angle - _zero_angle;
    _angle_vel = (_angle - _lst_angle) / dt;
    _lst_angle = _angle;
}

void Encoder::phase_z_trigger()
{
    if (_encoder == nullptr) {
        return;
    }
	_zero_count = _encoder->get_encoder_count();
	_zero_angle = (360.0f * _zero_count / _ppr) + _zero_angle_offset;   
}

