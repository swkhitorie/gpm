#include "qmc5883l.h"

qmc5883l::qmc5883l(enum QMC5883L_ORITENATION _ori, uint32_t _id, ESAF::ebus_i2cmaster *i2c) : orintation(_ori), _i2c(i2c) 
{
    device_id = _id;
    timestamp_start_block = 0;
    for (int i = 0; i < 6; i++) {
        mag[i % 3] = 0;
        mag_ga[i % 3] = 0;  
        rawmag[i] = 0;
    }
}

void qmc5883l::writeRegister(uint8_t regAddress, uint8_t data)
{
	if (_i2c != NULL) {
		ESAF::i2cMCmd command;
		uint8_t buf[2];
		buf[0] = regAddress;
		buf[1] = data;
		command.set(QMC5883L_ADDR, &buf[0], 2, 0, 0, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->write_through(&command, 1, ESAF::EDev::BLOCK);
	}
}

void qmc5883l::readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway)
{
	if (_i2c != NULL) {
		ESAF::i2cMCmd command;
		command.set(QMC5883L_ADDR, &regAddress, 1, buf, len, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->read_through(&command, 1, rwway);	
	}
}

uint8_t qmc5883l::checkid()
{
    uint8_t rcv_id = 0x00;
    readRegister(QMC5883L_ID_REG, &rcv_id, 1, ESAF::EDev::BLOCK);
    if (rcv_id != QMC5883L_CHIPID_VALUE) {
        return 0;
    }else {
        return 1;
    }
}

void qmc5883l::reset()
{
    writeRegister(QMC5883L_CONTROL2_REG, QMC5883L_CFGB_VALUE_REBOOT);
}

uint8_t qmc5883l::init()
{
    uint8_t config_check = 0;
	/* OSR = 512;RNG = 8G(0x1d RNG=8G);ODR=200Hz;MODE:待机模式*/
    writeRegister(QMC5883L_CONTROL1_REG, QMC5883L_CFGA_VALUE_CONTINUE);
    writeRegister(QMC5883L_ADDR_CFGC, QMC5883L_CFGC_VALUE);       //1.30Ga
    writeRegister(QMC5883L_ADDR_CFGD, QMC5883L_CFGD_VALUE);
    writeRegister(QMC5883L_SET_RESET_PERIOD, QMC5883L_PERIORC_VALUE);
    
    config_check = 0;
    readRegister(QMC5883L_CONTROL1_REG, &config_check, 1, ESAF::EDev::BLOCK);
    if (config_check != QMC5883L_CFGA_VALUE_CONTINUE) {
        return false;
    }
    
    config_check = 0;
    readRegister(QMC5883L_ADDR_CFGC, &config_check, 1, ESAF::EDev::BLOCK);
    if (config_check != QMC5883L_CFGC_VALUE) {
        return false;
    }
    
    config_check = 0;
    readRegister(QMC5883L_ADDR_CFGD, &config_check, 1, ESAF::EDev::BLOCK);
    if (config_check != QMC5883L_CFGD_VALUE) {
        return false;
    }

    config_check = 0;
    readRegister(QMC5883L_SET_RESET_PERIOD, &config_check, 1, ESAF::EDev::BLOCK);
    if (config_check != QMC5883L_PERIORC_VALUE) {
        return false;
    }
    
    return checkid();
}

void qmc5883l::update()
{
	readmag(&mag[0]);
    
    const float range_scale = 1000.0f / 3000.0f;

    // All those functions expect the mag field to be in milligauss.
    mag_ga[0] = static_cast<float>(mag[0]) * range_scale;
    mag_ga[1] = static_cast<float>(mag[1]) * range_scale;
    mag_ga[2] = static_cast<float>(mag[2]) * range_scale;

    #ifdef uOSB_PUBLISH_QMC5883L
        if (timestamp_start_block == 0) {
            timestamp_start_block = hrt_absolute_time();
        }
        _pub_sensor_mag.device_id = this->device_id;
        _pub_sensor_mag.timestamp = hrt_absolute_time();
        _pub_sensor_mag.timestamp_sample = hrt_absolute_time() - timestamp_start_block;
        _pub_sensor_mag.temperature = 0;
        _pub_sensor_mag.error_count = 0;

        if (orintation == QMC5883L_ORITENATION_NULL) {
            _pub_sensor_mag.x = mag_ga[0];
            _pub_sensor_mag.y = mag_ga[1];
            _pub_sensor_mag.z = mag_ga[2];           
        } else if (orintation == QMC5883L_ORITENATION_YAW_270) {
            _pub_sensor_mag.x = mag_ga[1];
            _pub_sensor_mag.y = -mag_ga[0];
            _pub_sensor_mag.z = mag_ga[2];
        }
        uorb_publish_topics<uORB::TOPIC_SENSOR_MAG>(&_pub_sensor_mag);
    #endif
}

void qmc5883l::readmag(int16_t *_mag)
{
	readRegister(QMC5883L_DATA_X_LSB, &rawmag[0], 6, ESAF::EDev::NONBLOCK);

    _mag[0] = (int16_t)((int16_t)(rawmag[1] << 8) | (rawmag[0]));
    _mag[1] = (int16_t)((int16_t)(rawmag[3] << 8) | (rawmag[2]));
    _mag[2] = (int16_t)((int16_t)(rawmag[5] << 8) | (rawmag[4]));
}

