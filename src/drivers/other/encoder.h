
#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "drv_exirq_trigger.hpp"
#include "drv_encoder_incremental.hpp"

class Encoder
{
public:
    Encoder() = delete;
    Encoder(uint32_t ppr, float offset, ESAF::drv_encoder_incremental *coder = nullptr);
    ~Encoder();

    void configcoder(ESAF::drv_encoder_incremental *coder);
    void update(float dt);

    void phase_z_trigger();

public:
    uint32_t _ppr;
    float _zero_angle_offset;
	int64_t _zero_count;
    float _zero_angle;
    int64_t _raw_count;
	float _raw_angle;

    int64_t _count;
	float _angle;
    float _lst_angle;
	float _angle_vel;

    ESAF::drv_encoder_incremental *_encoder;
};

#endif
