#ifndef LSM6DSR_H_
#define LSM6DSR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct __lsm6dsr_sensor {
	uint8_t chipid;
	float accel_scale;
	float gyro_scale;
	int32_t (*platform_write)(uint8_t reg, uint8_t *p, uint16_t len);
	int32_t (*platform_read)(uint8_t reg, uint8_t *p, uint16_t len);
} lsm6dsr_t;

int lsm6dsr_init(lsm6dsr_t *obj);
int lsm6dsr_read(lsm6dsr_t *obj, float *accel, float *gyro, float *temper);

#ifdef __cplusplus
}
#endif

#endif

