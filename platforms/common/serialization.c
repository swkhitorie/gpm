#include "serialization.h"

/*! @brief union for Convert memory
 *
 */
union Convert
{
    double data_double; /*!< tmp value for convert */
    float data_float[2];
    int64_t data_s64;
    int32_t data_s32[2];
    uint32_t data_u32[2];
    int16_t data_s16[4];
    uint16_t data_u16[4];
    uint8_t data_array[8];
} data_converter = { 0.0000000000000000 };

void double_to_bytes(double data, uint8_t array[], bool reverse)
{
    data_converter.data_double = data;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[7 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

void float_to_bytes(float data, uint8_t array[], bool reverse)
{
    data_converter.data_float[0] = data;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[3 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

void s64_to_bytes(int64_t data, uint8_t array[], bool reverse)
{
    data_converter.data_s64 = data;
    for (uint8_t i = 0; i < 8; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[7 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

void s32_to_bytes(int32_t data, uint8_t array[], bool reverse)
{
    data_converter.data_s32[0] = data;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[3 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

void s16_to_bytes(int16_t data, uint8_t array[], bool reverse)
{
    data_converter.data_s16[0] = data;
    for (uint8_t i = 0; i < 2; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[1 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

void u32_to_bytes(uint32_t data, uint8_t array[], bool reverse)
{
    data_converter.data_u32[0] = data;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[3 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

void u16_to_bytes(uint16_t data, uint8_t array[], bool reverse)
{
    data_converter.data_u16[0] = data;
    for (uint8_t i = 0; i < 2; i++)
    {
        if (reverse)
            array[i] = data_converter.data_array[1 - i];
        else
            array[i] = data_converter.data_array[i];
    }
}

double bytes_to_double(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[7 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_double;
}

float bytes_to_float(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[3 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_float[0];
}

int64_t bytes_to_s64(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[7 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_s64;
}

int32_t bytes_to_s32(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[3 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_s32[0];
}

int16_t bytes_to_s16(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[1 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_s16[0];
}

uint32_t bytes_to_u32(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[3 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_u32[0];
}

uint16_t bytes_to_u16(const uint8_t array[], bool reverse)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (reverse)
            data_converter.data_array[i] = array[1 - i];
        else
            data_converter.data_array[i] = array[i];
    }
    return data_converter.data_u16[0];
}
