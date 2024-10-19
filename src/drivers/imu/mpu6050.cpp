#include "mpu6050.h"

mpu6050::mpu6050(enum MPU6050_ORITENATION _ori, uint32_t _id, uint8_t mode, ESAF::ebus_i2cmaster *i2c, ESAF::ebus_spimaster *spi, 
	ESAF::drv_general_io *cs) : 
    orintation(_ori),
	drive_mode(mode),
	_i2c(i2c),
	_spi(spi),
	_cs(cs) 
{
    device_id = _id;
    timestamp_start_block = 0;
}

void mpu6050::wait_block(uint64_t time_us)
{
    uint64_t reset_time;
    reset_time = hrt_absolute_time();
    while (hrt_elapsed_time(&reset_time) < time_us); 
}

void mpu6050::writeRegister(uint8_t regAddress, uint8_t data)
{
	switch (drive_mode) {
	case _DRIVE_I2C:
		{
			if (_i2c != NULL) {
				ESAF::i2cMCmd command;
				uint8_t buf[2];
				buf[0] = regAddress;
				buf[1] = data;
				command.set(MPU_IIC_ADDR, &buf[0], 2, 0, 0, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
				_i2c->write_through(&command, 1, ESAF::EDev::BLOCK);
			}
			break;		
		}
	case _DRIVE_SPI:
		{
			if (_spi != NULL) {
				_cs->set(false);
                uint8_t addr = regAddress;// & 0x7F;
				_spi->write(&addr, 1);
				_spi->write(&data, 1);
				_cs->set(true);
			}
			break;
		}
	default: break;
	}
}

void mpu6050::readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway)
{
	switch (drive_mode) {
	case _DRIVE_I2C:
		{
			if (_i2c != NULL) {
				ESAF::i2cMCmd command;
				command.set(MPU_IIC_ADDR, &regAddress, 1, buf, len, true, this, ESAF::i2cMCmd::MEMADD_SIZE_8BIT);
				_i2c->read_through(&command, 1, rwway);	
			}
			break;		
		}
	case _DRIVE_SPI:
		{
			if (_spi != NULL) {
				_cs->set(false);
				uint8_t addr = regAddress | 0x80;
				_spi->write(&addr, 1);
                for(int i = 0; i < len; i++)
                    _spi->read(buf + i, 1);
				_cs->set(true);
			}
			break;
		}
	default: break;
	}
}

bool mpu6050::init()
{
    {
        writeRegister(MPU_PWR_MGMT1_REG, 0x80); //0x80
        wait_block((1e6f / 100));
        writeRegister(MPU_PWR_MGMT1_REG, 0x01); //0x80
        wait_block((1e6f / 100));
        writeRegister(MPU_SIGPATH_RST_REG, 0x07);
        wait_block((1e6f / 100));
        writeRegister(MPU_USER_CTRL_REG, 0x00);
        wait_block((1e6f / 100));
    }
    
    {
        //enable all sensor
        writeRegister(MPU_PWR_MGMT2_REG, 0x00);
        wait_block((1e6f / 100));
    }
    
    {
        //set sample rate and bandwith for acc ACCEL_FCHOICE_B = 1 A_DLPF_CFG = 0
        writeRegister(MPU_ACCEL_CFG_REG2, 0x08);  
        wait_block((1e6f / 100));
        writeRegister(MPU_SAMPLE_RATE_REG, 0x00);
        wait_block((1e6f / 100));
    }
    
    {
        //set sample rate and bandwith for gyro 0x01 / 0x07
        writeRegister(MPU_CFG_REG, 0x07);
        wait_block((1e6f / 100));
    }
    
    {
        //set sensitivity
        writeRegister(MPU_ACCEL_CFG_REG, 0x18);             // ¡À2g  
        wait_block((1e6f / 100));
        writeRegister(MPU_GYRO_CFG_REG, 0x18);              // ¡À2000dps 
        wait_block((1e6f / 100));
    }
    
    {
        if (drive_mode == _DRIVE_I2C) {
            //enable bypass mode
            writeRegister(MPU_INTBP_CFG_REG, 0x02);             // ¡À2g  
            wait_block((1e6f / 100));            
        }
    } 

    uint8_t config_check[2] = {0x00, 0x00};
    readRegister(MPU_GYRO_CFG_REG, &config_check[0], 1, ESAF::EDev::BLOCK);
    wait_block((1e6f / 100));
    readRegister(MPU_ACCEL_CFG_REG, &config_check[1], 1, ESAF::EDev::BLOCK);
    wait_block((1e6f / 100));  
    if (config_check[0] != 0x18 || config_check[1] != 0x18) return false;
    
	uint8_t tmpid = checkid();
    if (tmpid == MPU_ID_VALUE_1 || tmpid == MPU_ID_VALUE_2 || tmpid ==  MPU_ID_VALUE_3 || tmpid == MPU_ID_VALUE_4)
        return true;
    else
        return false;
}

uint8_t mpu6050::checkid()
{
	readRegister(MPU_DEVICE_ID_REG, &id, 1, ESAF::EDev::BLOCK);
	return id;
}

void mpu6050::readgyro(int16_t *gyro)
{
    readRegister(MPU_GYRO_XOUTH_REG, &rawgyro[0], 6, ESAF::EDev::NONBLOCK);
    gyro[0] = (rawgyro[0] << 8) | rawgyro[1];
    gyro[1] = (rawgyro[2] << 8) | rawgyro[3];
    gyro[2] = (rawgyro[4] << 8) | rawgyro[5];
}

void mpu6050::readacc(int16_t *acc)
{
    readRegister(MPU_ACCEL_XOUTH_REG, &rawacc[0], 6, ESAF::EDev::NONBLOCK);
    acc[0] = (rawacc[0] << 8) | rawacc[1];
    acc[1] = (rawacc[2] << 8) | rawacc[3];
    acc[2] = (rawacc[4] << 8) | rawacc[5];
}

void mpu6050::readtemper(int16_t *temp)
{
	readRegister(MPU_TEMP_OUTH_REG, &rawtemp[0], 2, ESAF::EDev::NONBLOCK);
	(*temp) = (int16_t)((rawtemp[0] << 8) | rawtemp[1]);
}

/*
 * temp_degC = (( temp_raw - roomtemp_offset) / temp_sensitivity) + 21degC
*/
void mpu6050::update()
{
    if (!_updated) {
        uint64_t us_now = hrt_absolute_time();
        if (us_now - _update_us > 0.05 * 1e6)
            _updated = true;
        return;
    }
    _updated = false;
    
	readRegister(MPU_ACCEL_XOUTH_REG, &rawall[0], 14, ESAF::EDev::NONBLOCK);

	rawacc[0] = rawall[0];
	rawacc[1] = rawall[1];
	rawacc[2] = rawall[2];
	rawacc[3] = rawall[3];
	rawacc[4] = rawall[4];
	rawacc[5] = rawall[5];

	rawtemp[0] = rawall[6];
	rawtemp[1] = rawall[7];

	rawgyro[0] = rawall[8];
	rawgyro[1] = rawall[9];
	rawgyro[2] = rawall[10];
	rawgyro[3] = rawall[11];
	rawgyro[4] = rawall[12];
	rawgyro[5] = rawall[13];	

    gyro_origin[0] = (rawgyro[0] << 8) | rawgyro[1];
    gyro_origin[1] = (rawgyro[2] << 8) | rawgyro[3];
    gyro_origin[2] = (rawgyro[4] << 8) | rawgyro[5];
    
    acc_origin[0] = (rawacc[0] << 8) | rawacc[1];
    acc_origin[1] = (rawacc[2] << 8) | rawacc[3];
    acc_origin[2] = (rawacc[4] << 8) | rawacc[5];
    
	temper = (rawtemp[0] << 8) | rawtemp[1];
    
    for (int i = 0; i < 3; i++) {
        gyro_rad[i] = math::radians( gyro_origin[i] * Gyro_2000An_SCALE);
        accel_m_s2[i] =  acc_origin[i] * ACC_16G_SCALE * GRAVITY_MSS;
    }

    #ifdef uOSB_PUBLISH_MPU6500
        if (timestamp_start_block == 0) {
            timestamp_start_block = hrt_absolute_time();
        }
        
        _pub_sensor_accel.device_id = this->device_id;
        _pub_sensor_accel.error_count = 0;
        _pub_sensor_accel.timestamp = hrt_absolute_time();
        _pub_sensor_accel.timestamp_sample = hrt_absolute_time() - timestamp_start_block;
        _pub_sensor_accel.temperature = 36.53f + (temper / 340.0f);
        _pub_sensor_accel.samples = 1;

        _pub_sensor_gyro.device_id = this->device_id;
        _pub_sensor_gyro.error_count = 0;
        _pub_sensor_gyro.timestamp = hrt_absolute_time();
        _pub_sensor_gyro.timestamp_sample = hrt_absolute_time() - timestamp_start_block;
        _pub_sensor_gyro.temperature = 36.53f + (temper / 340.0f);   
        _pub_sensor_gyro.samples = 1;
        
        if (orintation == MPU6050_ORITENATION_NULL) {
            _pub_sensor_accel.x = accel_m_s2[0];
            _pub_sensor_accel.y = accel_m_s2[1];
            _pub_sensor_accel.z = accel_m_s2[2];
            _pub_sensor_gyro.x = gyro_rad[0];
            _pub_sensor_gyro.y = gyro_rad[1];
            _pub_sensor_gyro.z = gyro_rad[2];            
        } else if (orintation == MPU6050_ORITENATION_YAW_270) {
            _pub_sensor_accel.x = accel_m_s2[1];
            _pub_sensor_accel.y = -accel_m_s2[0];
            _pub_sensor_accel.z = accel_m_s2[2];
            _pub_sensor_gyro.x = gyro_rad[1];
            _pub_sensor_gyro.y = -gyro_rad[0];
            _pub_sensor_gyro.z = gyro_rad[2];  
        }
        
        uorb_publish_topics<uORB::TOPIC_SENSOR_ACCEL>(&_pub_sensor_accel);
        uorb_publish_topics<uORB::TOPIC_SENSOR_GYRO>(&_pub_sensor_gyro);
    #endif
}

