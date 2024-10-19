#ifndef __HMC5883L_H_
#define __HMC5883L_H_

#include "ebus_i2cmaster.hpp"
#include "i2cMCmd.hpp"
#include "platform_defines.h"

#define uOSB_PUBLISH_HMC5883L
#ifdef uOSB_PUBLISH_HMC5883L
    #include "uORB/uORB_topic_define.hpp"
#endif

#define	HMC5883L_Addr                       0x3C	 
#define HMC5883L_ConfigA                    0x00
#define HMC5883L_ConfigB                    0x01
#define HMC5883L_ModeReg                    0x02
#define HMC5883L_Output_X_MSB               0x03
#define HMC5883L_Output_X_LSB               0x04
#define HMC5883L_Output_Z_MSB               0x05
#define HMC5883L_Output_Z_LSB               0x06
#define HMC5883L_Output_Y_MSB               0x07
#define HMC5883L_Output_Y_LSB               0x08
#define HMC5883L_StatusRegister             0x09
#define HMC5883L_ID_A                       0x0A
#define HMC5883L_ID_B                       0x0B
#define HMC5883L_ID_C                       0x0C

/*	HMC5883L DataTransfer HMC5883L_ConfigurationRegisterB */
#define HMC5883L_GA_LSB0                    1370				//0x00    ¡À0.88Ga
#define HMC5883L_GA_LSB1                    1090				//0x20    ¡À1.30Ga
#define HMC5883L_GA_LSB2                    820					//0x40    ¡À1.90Ga
#define HMC5883L_GA_LSB3                    660					//0x60    ¡À2.50Ga
#define HMC5883L_GA_LSB4                    440					//0x80    ¡À4.00Ga
#define HMC5883L_GA_LSB5                    390					//0xA0    ¡À4.70Ga
#define HMC5883L_GA_LSB6                    330					//0xC0    ¡À5.66Ga
#define HMC5883L_GA_LSB7                    230					//0xE0    ¡À8.10Ga

enum HMC5883L_ORITENATION {
    HMC5883L_ORITENATION_NULL = 0x00,
    HMC5883L_ORITENATION_YAW_270, //pilot x -> sensor y, pilot y -> sensor -x
};

class hmc5883l : public i2cDevice
{
public:
    hmc5883l() = delete;
    hmc5883l(enum HMC5883L_ORITENATION _ori, uint32_t _id, ESAF::ebus_i2cmaster *i2c);
    ~hmc5883l();

	uint8_t checkid();
	uint8_t init();
	void update();
	void readmag(int16_t *_mag);

public:
    
	void writeRegister(uint8_t regAddress, uint8_t data);
	void readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway); 

    uint32_t device_id;
    uint64_t timestamp_start_block;

    uint8_t orintation;

    uint8_t rawmag[6];
    uint8_t id[3];
    int16_t mag[3];
    float mag_ga[3];
	ESAF::ebus_i2cmaster *_i2c;

    #ifdef uOSB_PUBLISH_HMC5883L
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_MAG>::type _pub_sensor_mag;
    #endif
};

#endif
