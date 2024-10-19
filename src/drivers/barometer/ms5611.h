#ifndef __MS5611_H_
#define __MS5611_H_

#include "ebus_i2cmaster.hpp"
#include "i2cMCmd.hpp"
#include "platform_defines.h"
/*
	Reset
	Read PROM(128 bit of calibration words)
	D1 conversion
	D2 conversion
	Read ADC result(24 bit pressure/temperature)
	1 mbar = 0.1 Kpa
	Maximum Values for calculation results:
	Pmin = 10mbar   Pmax = 1200mbar
	Tmin = -40°C  Tmax = 85°C  Tref = 20°C 
*/
#define MS5611_ADDR                     0xEE
#define MS5611_D1                       0x40			
#define MS5611_D2                       0x50			
#define MS5611_RESET                    0x1E
#define MS5611_D1D2_SIZE                0x03	
// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256                  0x00
#define MS5611_OSR_512                  0x02
#define MS5611_OSR_1024                 0x04
#define MS5611_OSR_2048                 0x06
#define MS5611_OSR_4096                 0x08
#define MS5611_PROM_BASE_ADDR           0xA2
#define MS5611_PROM_REG_COUNT           0x06 
#define MS5611_PROM_REG_SIZE            0x02 

class ms5611 : public i2cDevice
{
public:
    ms5611() = delete;
    ms5611(ESAF::ebus_i2cmaster *i2c);
    ~ms5611();

    void singlecommand(uint8_t regAddress);
	void writeRegister(uint8_t regAddress, uint8_t data);
	void readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway);

	uint8_t init();
	void reset();
    void update();
    void cal_temperature();
    void cal_press();

public:
    uint32_t Raw_P, Raw_T;
    int32_t dT;
    int32_t TEMP;
    int64_t OFF, OFF2;
    int64_t SENS, SENS2;
    int64_t P;
    int64_t T2; 

    float _press;
    float _temperture;
    float _altitude;

    bool _mConvertPress;
    uint8_t _mRomData[12];
    uint16_t _mC[6];
    uint8_t _mRawT[3];
    uint8_t _mRawP[3];

    ESAF::ebus_i2cmaster *_i2c;
};

#endif
