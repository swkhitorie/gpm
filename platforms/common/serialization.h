
#ifndef __SERIALIZATION_H_
#define __SERIALIZATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
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

#ifdef __cplusplus
}
#endif

#endif  // serialize_H
