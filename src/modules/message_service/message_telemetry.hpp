
#ifndef __MESSAGE_TELEMETRY_H_
#define __MESSAGE_TELEMETRY_H_

#include "stdint.h"

namespace ESAF
{
namespace Telemetry
{

/*!
 * @brief enum MessageName is the interface for user to create create packages and access data
 * @note Please see telemetry_doc.hpp for detailed documentation.
 */
typedef enum
{
    MSG_UINT8_T,
    MSG_INT8_T,
    MSG_UINT16_T,
    MSG_INT16_T,
    MSG_UINT32_T,
    MSG_INT32_T,
    MSG_FLOAT_T,
    MSG_DOUBLE_T,

    MSG_IMU,
    MSG_EULER,

    TOTAL_MSG,
} MessageName;

#pragma pack(1)
typedef struct Vector3d
{
    float x;
    float y;
    float z;
} Vector3d;

/*!
 * @brief struct for MSG_IMU
 */
typedef struct IMU
{
    Vector3d gyro;
    Vector3d accel;
} IMU;

/*!
 * @brief struct for MSG_EULER
 */
typedef struct Euler
{
    Vector3d angle;
} Euler;

#pragma pack()

/*! @brief template struct maps a message name to the corresponding data type
 */
template <MessageName T>
struct msgTypeMap
{
    typedef void type;
};

// clang-format off
template <> struct msgTypeMap<MSG_UINT8_T              > { typedef uint8_t  type;};
template <> struct msgTypeMap<MSG_INT8_T               > { typedef int8_t   type;};
template <> struct msgTypeMap<MSG_UINT16_T             > { typedef uint16_t type;};
template <> struct msgTypeMap<MSG_INT16_T              > { typedef int16_t  type;};
template <> struct msgTypeMap<MSG_UINT32_T             > { typedef uint32_t type;};
template <> struct msgTypeMap<MSG_INT32_T              > { typedef int32_t  type;};
template <> struct msgTypeMap<MSG_FLOAT_T              > { typedef float    type;};
template <> struct msgTypeMap<MSG_DOUBLE_T             > { typedef double   type;};
template <> struct msgTypeMap<MSG_IMU                  > { typedef IMU      type;};
template <> struct msgTypeMap<MSG_EULER                > { typedef Euler    type;};

}  // namespace Telemetry
}  // namespace ESAF


#endif  // message_telemetry_H
