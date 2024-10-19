#include "ms5611.h"
#include <math.h>
// #include "utility/log.hpp"
ms5611::ms5611(ESAF::ebus_i2cmaster *i2c) :
    Raw_P(0),
    Raw_T(0),
    dT(0),
    TEMP(0),
    OFF(0),
    OFF2(0),
    SENS(0),
    SENS2(0),
    P(0),
    T2(0),
    _press(0.0f),
    _temperture(0.0f),
    _altitude(0.0f),
    _mConvertPress(false),
	_i2c(i2c) 
{
    for (int i = 0; i < 3; i++) {
        _mRawT[i] = 0;
        _mRawP[i] = 0;
    }
    for (int i = 0; i < 6; i++) {
        _mC[i] = 0;
    }
    for (int i = 0; i < 12; i++) {
        _mRomData[i] = 0;
    }
}
ms5611::~ms5611() {}

void ms5611::singlecommand(uint8_t regAddress)
{
	if (_i2c != NULL) {
	    ESAF::i2cMCmd cmd;
	    uint8_t reset_com = regAddress;
	    cmd.set(MS5611_ADDR, &reset_com, 1, 0, 0, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
	    _i2c->write_through(&cmd, 1, ESAF::EDev::NONBLOCK);
	}
}

void ms5611::writeRegister(uint8_t regAddress, uint8_t data)
{
	if (_i2c != NULL) {
		ESAF::i2cMCmd cmd;
		uint8_t buf[2];
		buf[0] = regAddress;
		buf[1] = data;
		cmd.set(MS5611_ADDR, &buf[0], 2, 0, 0, false, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->write_through(&cmd, 1, ESAF::EDev::BLOCK);
	}
}

void ms5611::readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway)
{
	if (_i2c != NULL) {
		ESAF::i2cMCmd cmd;
		cmd.set(MS5611_ADDR, &regAddress, 1, buf, len, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->read_through(&cmd, 1, rwway);	
	}
}

void ms5611::reset()
{
	singlecommand(MS5611_RESET);
	for (uint32_t i = 0; i < 100000; i++);
}

uint8_t ms5611::init()
{
	uint8_t i;
	readRegister(MS5611_PROM_BASE_ADDR, &_mRomData[0], 2, ESAF::EDev::BLOCK);
	readRegister(MS5611_PROM_BASE_ADDR + 2, &_mRomData[2], 2, ESAF::EDev::BLOCK);
	readRegister(MS5611_PROM_BASE_ADDR + 4, &_mRomData[4], 2, ESAF::EDev::BLOCK);
	readRegister(MS5611_PROM_BASE_ADDR + 6, &_mRomData[6], 2, ESAF::EDev::BLOCK);
	readRegister(MS5611_PROM_BASE_ADDR + 8, &_mRomData[8], 2, ESAF::EDev::BLOCK);
	readRegister(MS5611_PROM_BASE_ADDR + 10, &_mRomData[10], 2, ESAF::EDev::BLOCK);

	for (i = 0; i < 6; i++) {
		if (_mRomData[i * 2] == 0x00 && _mRomData[i * 2 + 1] == 0x00)
			break;
		_mC[i] = ((uint16_t)((_mRomData[i * 2] << 8) | _mRomData[i * 2 + 1]));
	}
	
	if (i != 6)
		return 1;
	else 
		return 0;
}

void ms5611::update()
{
    if (!_updated) {
        uint64_t us_now = hrt_absolute_time();
        if (us_now - _update_us > 0.5 * 1e6)
            _updated = true;
        return;
    }
    _updated = false;

	if (_mConvertPress) {
		readRegister(0, &_mRawT[0], 3, ESAF::EDev::NONBLOCK);
		singlecommand(MS5611_D1 + MS5611_OSR_4096);
	}else {
		readRegister(0, &_mRawP[0], 3, ESAF::EDev::NONBLOCK);
		singlecommand(MS5611_D2 + MS5611_OSR_4096);
	}
	_mConvertPress = !_mConvertPress;
	cal_temperature();
	cal_press();
    
	_press = (float)P / 100;				//to mbar
	_temperture = (float)TEMP / 100.0f;		//to dot
	_altitude = (44330.0f * (1.0f - powf((float)_press / 101325.0f, 0.190295f))) / 100.0f;
    
}

void ms5611::cal_temperature()
{
	Raw_T = (((uint32_t)(_mRawT[0] << 16)) | ((uint32_t)(_mRawT[1]) << 8) | _mRawT[2]);
	dT = (int32_t)(Raw_T - uint32_t(_mC[4] << 8));
	TEMP = 2000 + ((dT * _mC[5]) >> 23);
	if (TEMP < 2000) {
		T2 = ((dT * dT) >> 31);	
		OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS2 = (5 * (TEMP - 2000) * (TEMP - 2000)) / 4;
		if (TEMP < -1500) {
			OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
			SENS2 = SENS2 + ((11 * (TEMP + 1500) * (TEMP + 1500)) >> 1);
		}
	}else {
		T2 = 0;	
		OFF2= 0;
		SENS2= 0;
	}
	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;
}

void ms5611::cal_press()
{
	Raw_P = (((uint32_t)(_mRawP[0] << 16)) | ((uint32_t)(_mRawP[1]) << 8) | _mRawP[2]);
	OFF = ((int64_t)(_mC[1]) << 16) + (((int64_t)(_mC[3] * dT)) >> 7);
	SENS = ((int64_t)(_mC[0]) << 15) + (((int64_t)(_mC[2]) * dT) >> 8);
	P = (((Raw_P * SENS) >> 21) - OFF) >> 15;
}
