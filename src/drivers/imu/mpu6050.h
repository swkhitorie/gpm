#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "ebus_i2cmaster.hpp"
#include "ebus_spimaster.hpp"
#include "i2cMCmd.hpp"
#include "spiMCmd.hpp"
#include "drv_general_io.hpp"
#include <mathlib/mathlib.h>
#include "platform_defines.h"

#define uOSB_PUBLISH_MPU6500
#ifdef uOSB_PUBLISH_MPU6500
    #include "uORB/uORB_topic_define.hpp"
#endif

#define MPU_SELF_TESTX_REG				0X0D	
#define MPU_SELF_TESTY_REG				0X0E	
#define MPU_SELF_TESTZ_REG				0X0F	
#define MPU_SELF_TESTA_REG				0X10	
#define MPU_SAMPLE_RATE_REG				0X19	
#define MPU_CFG_REG						0X1A
#define MPU_GYRO_CFG_REG				0X1B	
#define MPU_ACCEL_CFG_REG				0X1C
#define MPU_ACCEL_CFG_REG2				0X1D
#define MPU_MOTION_DET_REG				0X1F	
#define MPU_FIFO_EN_REG					0X23	
#define MPU_I2CMST_CTRL_REG				0X24	
#define MPU_I2CSLV0_ADDR_REG			0X25	
#define MPU_I2CSLV0_REG					0X26	
#define MPU_I2CSLV0_CTRL_REG			0X27	
#define MPU_I2CSLV1_ADDR_REG			0X28	
#define MPU_I2CSLV1_REG					0X29	
#define MPU_I2CSLV1_CTRL_REG			0X2A	
#define MPU_I2CSLV2_ADDR_REG			0X2B	
#define MPU_I2CSLV2_REG					0X2C	
#define MPU_I2CSLV2_CTRL_REG			0X2D	
#define MPU_I2CSLV3_ADDR_REG			0X2E	
#define MPU_I2CSLV3_REG					0X2F	
#define MPU_I2CSLV3_CTRL_REG			0X30	
#define MPU_I2CSLV4_ADDR_REG			0X31	
#define MPU_I2CSLV4_REG					0X32	
#define MPU_I2CSLV4_DO_REG				0X33	
#define MPU_I2CSLV4_CTRL_REG			0X34	
#define MPU_I2CSLV4_DI_REG				0X35	
#define MPU_I2CMST_STA_REG				0X36	
#define MPU_INTBP_CFG_REG				0X37	
#define MPU_INT_EN_REG					0X38	
#define MPU_INT_STA_REG					0X3A	
#define MPU_ACCEL_XOUTH_REG				0X3B	
#define MPU_ACCEL_XOUTL_REG				0X3C	
#define MPU_ACCEL_YOUTH_REG				0X3D	
#define MPU_ACCEL_YOUTL_REG				0X3E	
#define MPU_ACCEL_ZOUTH_REG				0X3F	
#define MPU_ACCEL_ZOUTL_REG				0X40	
#define MPU_TEMP_OUTH_REG				0X41	
#define MPU_TEMP_OUTL_REG				0X42
#define MPU_GYRO_XOUTH_REG				0X43	
#define MPU_GYRO_XOUTL_REG				0X44	
#define MPU_GYRO_YOUTH_REG				0X45	
#define MPU_GYRO_YOUTL_REG				0X46	
#define MPU_GYRO_ZOUTH_REG				0X47	
#define MPU_GYRO_ZOUTL_REG				0X48	

#define MPU_I2CSLV0_DO_REG				0X63	
#define MPU_I2CSLV1_DO_REG				0X64	
#define MPU_I2CSLV2_DO_REG				0X65	
#define MPU_I2CSLV3_DO_REG				0X66	
#define MPU_I2CMST_DELAY_REG			0X67	
#define MPU_SIGPATH_RST_REG				0X68	
#define MPU_MDETECT_CTRL_REG			0X69	
#define MPU_USER_CTRL_REG				0X6A	
#define MPU_PWR_MGMT1_REG				0X6B	
#define MPU_PWR_MGMT2_REG				0X6C	
#define MPU_FIFO_CNTH_REG				0X72	
#define MPU_FIFO_CNTL_REG				0X73	
#define MPU_FIFO_RW_REG					0X74	
#define MPU_DEVICE_ID_REG				0X75	

#define MPU_PRODUCT_ADDR				0x98
#define MPU_IIC_ADDR    				0xD0
#define ACC_2G_SCALE   					(float)((2 * 2) / 65536.0)
#define ACC_4G_SCALE   					(float)((2 * 4) / 65536.0)
#define ACC_8G_SCALE   					(float)((2 * 8) / 65536.0)
#define ACC_16G_SCALE   				(float)((2 * 16) / 65536.0)
#define Gyro_250An_SCALE 				(float)((2 * 250.0) / 65536.0)
#define Gyro_500An_SCALE 				(float)((2 * 500.0) / 65536.0)
#define Gyro_1000An_SCALE 				(float)((2 * 1000.0) / 65536.0)
#define Gyro_2000An_SCALE 				(float)((2 * 2000.0) / 65536.0)

#define MPU_ID_VALUE_1					0x68
#define MPU_ID_VALUE_2					0x71
#define MPU_ID_VALUE_3					0x73
#define MPU_ID_VALUE_4					0x70

enum MPU6050_ORITENATION {
    MPU6050_ORITENATION_NULL = 0x00,
    MPU6050_ORITENATION_YAW_270, //pilot x -> sensor y, pilot y -> sensor -x
};

class mpu6050 : public i2cDevice
{
public:
    mpu6050() = delete;
    mpu6050(enum MPU6050_ORITENATION _ori ,uint32_t _id, uint8_t mode, ESAF::ebus_i2cmaster *i2c = nullptr, 
    			ESAF::ebus_spimaster *spi = nullptr, ESAF::drv_general_io *cs = nullptr);
    ~mpu6050() = default;

	enum DRIVE_MODE {
		_DRIVE_I2C,
		_DRIVE_SPI,
	};

	uint8_t checkid();
	bool init();         //need wait block 100ms at least per command
	
	void update();
	
	void readgyro(int16_t *gyro);
	void readacc(int16_t *acc);
	void readtemper(int16_t *temp);

	void writeRegister(uint8_t regAddress, uint8_t data);
	void readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway);

public:
    void wait_block(uint64_t time_us);

    uint32_t device_id;
    uint64_t timestamp_start_block;

    uint8_t orintation;

	uint8_t id;
	int16_t temper;
	uint8_t drive_mode;

	uint8_t rawgyro[6];
	uint8_t rawtemp[2];
	uint8_t rawacc[6];
	uint8_t rawall[14];
	
	int16_t gyro_origin[3];
	int16_t acc_origin[3];

    float gyro_rad[3];
    float accel_m_s2[3];

	ESAF::ebus_i2cmaster *_i2c;
	ESAF::ebus_spimaster *_spi;
	ESAF::drv_general_io *_cs;

    #ifdef uOSB_PUBLISH_MPU6500
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_ACCEL>::type _pub_sensor_accel;
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_GYRO>::type _pub_sensor_gyro;
    #endif
};

/*
	writeRegister(MPU_PWR_MGMT1_REG, 0x80);
	for (int i = 0; i < 5000; i++);
	writeRegister(MPU_PWR_MGMT1_REG, 0x00);
	writeRegister(MPU_GYRO_CFG_REG, 0x18);              // ¡À2000dps
	writeRegister(MPU_ACCEL_CFG_REG, 0x00);             // ¡À2g
	writeRegister(MPU_SAMPLE_RATE_REG, 0x00);
	writeRegister(MPU_CFG_REG, 0x00);
	writeRegister(MPU_INT_EN_REG, 0x00);
	writeRegister(MPU_USER_CTRL_REG, 0x00);
	writeRegister(MPU_FIFO_EN_REG, 0x00);
	writeRegister(MPU_INTBP_CFG_REG, 0x82);
	writeRegister(MPU_PWR_MGMT1_REG, 0x01);
	writeRegister(MPU_PWR_MGMT2_REG, 0x00);
	writeRegister(MPU_SAMPLE_RATE_REG, 0x00);
*/
#endif
