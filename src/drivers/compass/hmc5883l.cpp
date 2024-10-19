#include "hmc5883l.h"

hmc5883l::hmc5883l(enum HMC5883L_ORITENATION _ori, uint32_t _id, ESAF::ebus_i2cmaster *i2c) : orintation(_ori), _i2c(i2c) 
{
    device_id = _id;
    timestamp_start_block = 0;
    for (int i = 0; i < 6; i++) {
        id[i % 3] = 0;
        mag[i % 3] = 0;
        mag_ga[i % 3] = 0;  
        rawmag[i] = 0;
    }
}
hmc5883l::~hmc5883l() {}

void hmc5883l::writeRegister(uint8_t regAddress, uint8_t data)
{
	if (_i2c != NULL) {
		ESAF::i2cMCmd command;
		uint8_t buf[2];
		buf[0] = regAddress;
		buf[1] = data;
		command.set(HMC5883L_Addr, &buf[0], 2, 0, 0, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->write_through(&command, 1, ESAF::EDev::BLOCK);
	}
}

void hmc5883l::readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway)
{
	if (_i2c != NULL) {
		ESAF::i2cMCmd command;
		command.set(HMC5883L_Addr, &regAddress, 1, buf, len, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
		_i2c->read_through(&command, 1, rwway);	
	}
}

uint8_t hmc5883l::checkid()
{
    readRegister(HMC5883L_ID_A, &id[0], 3, ESAF::EDev::BLOCK);
    if (id[0] == 'H' && id[1] == '4' && id[2] == '3') {
        return 1;
    }else {
        return 0;
    }
}

uint8_t hmc5883l::init()
{
    uint8_t checkreg;
    writeRegister(HMC5883L_ConfigA, 0x78);
    writeRegister(HMC5883L_ConfigB, 0x00 | (0x02 << 5));       //1.90Ga
    writeRegister(HMC5883L_ModeReg, 0x00);
    
    readRegister(HMC5883L_ConfigA, &checkreg, 1, ESAF::EDev::BLOCK);
    if (checkreg != 0x78) return 0;
    
    readRegister(HMC5883L_ConfigB, &checkreg, 1, ESAF::EDev::BLOCK);
    if (checkreg != 0x64) return 0; 
    
    readRegister(HMC5883L_ModeReg, &checkreg, 1, ESAF::EDev::BLOCK);
    if (checkreg != 0x00) return 0;
    
    return checkid();
}

void hmc5883l::update()
{
    if (!_updated) {
        uint64_t us_now = hrt_absolute_time();
        if (us_now - _update_us > 0.02 * 1e6)
            _updated = true;
        return;
    }
    
    _updated = false;
    
	readmag(&mag[0]);
    const float range_scale = 1000.0f / HMC5883L_GA_LSB2;
    
    // milligauss
    mag_ga[0] = static_cast<float>(mag[0]) * range_scale;
    mag_ga[1] = static_cast<float>(mag[1]) * range_scale;
    mag_ga[2] = static_cast<float>(mag[2]) * range_scale;
    
    #ifdef uOSB_PUBLISH_HMC5883L
        if (timestamp_start_block == 0) {
            timestamp_start_block = hrt_absolute_time();
        }
        _pub_sensor_mag.device_id = this->device_id;
        _pub_sensor_mag.timestamp = hrt_absolute_time();
        _pub_sensor_mag.timestamp_sample = hrt_absolute_time() - timestamp_start_block;
        _pub_sensor_mag.temperature = 0;
        _pub_sensor_mag.error_count = 0;
        
        if (orintation == HMC5883L_ORITENATION_NULL) {
            _pub_sensor_mag.x = mag_ga[0];
            _pub_sensor_mag.y = mag_ga[1];
            _pub_sensor_mag.z = mag_ga[2];         
        } else if (orintation == HMC5883L_ORITENATION_YAW_270) {
            _pub_sensor_mag.x = mag_ga[1];
            _pub_sensor_mag.y = -mag_ga[0];
            _pub_sensor_mag.z = mag_ga[2];
        }
        
        uorb_publish_topics<uORB::TOPIC_SENSOR_MAG>(&_pub_sensor_mag);
    #endif
}

void hmc5883l::readmag(int16_t *_mag)
{
	readRegister(HMC5883L_Output_X_MSB, &rawmag[0], 6, ESAF::EDev::NONBLOCK);

    _mag[0] = (int16_t)((int16_t)(rawmag[0] << 8) | (rawmag[1]));
    _mag[1] = (int16_t)((int16_t)(rawmag[2] << 8) | (rawmag[3]));
    _mag[2] = (int16_t)((int16_t)(rawmag[4] << 8) | (rawmag[5]));
}
