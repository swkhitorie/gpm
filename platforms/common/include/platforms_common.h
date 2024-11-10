/**
 * @file platforms_common.h
 *
 * Generally used magic defines and tools function
*/
#ifndef __PLAFTORM_DEFINE_H_
#define __PLAFTORM_DEFINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define USE_COMMON_UTILS

typedef uint64_t hrt_abstime;

/* Math macro's for float literals. Do not use M_PI et al as they aren't
 * defined (neither C nor the C++ standard define math constants) */
#define M_E_F               2.71828183f
#define M_LOG2E_F           1.44269504f
#define M_LOG10E_F          0.43429448f
#define M_LN2_F             0.69314718f
#define M_LN10_F            2.30258509f
#define M_PI_F              3.14159265f
#define M_TWOPI_F           6.28318531f
#define M_PI_2_F            1.57079632f
#define M_PI_4_F            0.78539816f
#define M_3PI_4_F           2.35619449f
#define M_SQRTPI_F          1.77245385f
#define M_1_PI_F            0.31830989f
#define M_2_PI_F            0.63661977f
#define M_2_SQRTPI_F        1.12837917f
#define M_DEG_TO_RAD_F      0.0174532925f
#define M_RAD_TO_DEG_F      57.2957795f
#define M_SQRT2_F           1.41421356f
#define M_SQRT1_2_F         0.70710678f
#define M_LN2LO_F           1.90821484E-10f
#define M_LN2HI_F           0.69314718f
#define M_SQRT3_F           1.73205081f
#define M_IVLN10_F          0.43429448f	// 1 / log(10)
#define M_LOG2_E_F          0.69314718f
#define M_INVLN2_F          1.44269504f	// 1 / log(2)

/* The M_PI, as stated above, is not C standard. If you need it and
 * it isn't in your math.h file then you can use this instead. */
#define M_PI_PRECISE        3.141592653589793238462643383279502884
#define M_DEG_TO_RAD        0.017453292519943295
#define M_RAD_TO_DEG        57.295779513082323
#define GRAVITY_MSS         9.80665f

extern hrt_abstime hrt_absolute_time();
static inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then){return hrt_absolute_time() - *then;}

#ifdef USE_COMMON_UTILS

#define DEBUG_PRINT_ENABLE (1)
#define DEBUG_PRINT_BUFFER (128)

void print(const char *fmt, ...);
void prtsendbytes(const uint8_t *pdata, uint16_t len);

typedef struct __devbuf {
	uint8_t *buf;
	uint16_t capacity;
	uint16_t size;
	uint16_t idx_out;
	uint16_t idx_in;
	uint16_t overflow;
} devbuf_t;

uint16_t devbuf_size(devbuf_t *obj);
uint16_t devbuf_free(devbuf_t *obj);
uint16_t devbuf_overflow(devbuf_t *obj);
bool devbuf_is_empty(devbuf_t *obj);
bool devbuf_is_full(devbuf_t *obj);
void devbuf_reset(devbuf_t *obj);
void devbuf_init(devbuf_t *obj, uint8_t *p, uint16_t capacity);

uint16_t devbuf_write(devbuf_t *obj, const uint8_t *p, uint16_t len);
uint16_t devbuf_read(devbuf_t *obj, uint8_t *p, uint16_t len);
uint16_t devbuf_query(devbuf_t *obj, uint16_t offset, uint8_t *p, uint16_t len);

/*! @brief Serialize API:
 *  provide conversion between byte streams and data
 */

/*! @brief turn double value to bytes
 * @param data double value
 * @param array[] output bytes
 * @param reverse false - little endian, true - big endian
 */
void double_to_bytes(double data, uint8_t array[], bool reverse);
void float_to_bytes(float data, uint8_t array[], bool reverse);
void s64_to_bytes(int64_t data, uint8_t array[], bool reverse);
void s32_to_bytes(int32_t data, uint8_t array[], bool reverse);
void s16_to_bytes(int16_t data, uint8_t array[], bool reverse);
void u32_to_bytes(uint32_t data, uint8_t array[], bool reverse);
void u16_to_bytes(uint16_t data, uint8_t array[], bool reverse);

/*! @brief turn bytes to double value
 * @param array[] input bytes
 * @param reverse false - little endian, true - big endian
 * @return double value
 */
double bytes_to_double(const uint8_t array[], bool reverse);
float bytes_to_float(const uint8_t array[], bool reverse);
int64_t bytes_to_s64(const uint8_t array[], bool reverse);
int32_t bytes_to_s32(const uint8_t array[], bool reverse);
int16_t bytes_to_s16(const uint8_t array[], bool reverse);
uint32_t bytes_to_u32(const uint8_t array[], bool reverse);
uint16_t bytes_to_u16(const uint8_t array[], bool reverse);

#endif // Macro USE_COMMON_UTILS

#ifdef __cplusplus
}
#endif

#endif
