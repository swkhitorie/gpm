#ifndef __QMC5883L_H_
#define __QMC5883L_H_

#include "ebus_i2cmaster.hpp"
#include "i2cMCmd.hpp"
#include "uORB/uORB_topic_define.hpp"

#define uOSB_PUBLISH_QMC5883L
#ifdef uOSB_PUBLISH_QMC5883L
    #include "uORB/uORB_topic_define.hpp"
#endif

#define QMC5883L_CHIPID_VALUE				0xFF
#define	QMC5883L_ADDR						0x1A

#define QMC5883L_DATA_X_LSB					0x00
#define QMC5883L_DATA_X_MSB					0x01
#define QMC5883L_DATA_Y_LSB					0x02
#define QMC5883L_DATA_Y_MSB					0x03
#define QMC5883L_DATA_Z_LSB					0x04
#define QMC5883L_DATA_Z_MSB					0x05
#define QMC5883L_STATUS_REG					0x06
#define QMC5883L_TEMP_LSB					0x07
#define QMC5883L_TEMP_MSB					0x08
#define QMC5883L_CONTROL1_REG				0x09
#define QMC5883L_CONTROL2_REG				0x0A		//控制寄存器2	读写-BIT7为复位
#define QMC5883L_SET_RESET_PERIOD			0x0B		//时间寄存器，设置为0x01
#define QMC5883L_ID_REG						0x0D

#define	QMC5883L_ADDR_CFGC					0x20
#define	QMC5883L_ADDR_CFGD					0x21

#define QMC5883L_CFGA_OSR_512				(0 << 7) | (0 << 6)
#define QMC5883L_CFGA_OSR_256               (0 << 7) | (1 << 6)
#define QMC5883L_CFGA_OSR_128               (1 << 7) | (0 << 6)
#define QMC5883L_CFGA_OSR_64                (1 << 7) | (1 << 6)
#define QMC5883L_CFGA_RNG_2G                (0 << 5) | (0 << 4)
#define QMC5883L_CFGA_RNG_8G                (0 << 5) | (1 << 4)
#define QMC5883L_CFGA_ODR_10HZ              (0 << 3) | (0 << 2)
#define QMC5883L_CFGA_ODR_50HZ              (0 << 3) | (1 << 2)
#define QMC5883L_CFGA_ODR_100HZ             (1 << 3) | (0 << 2)
#define QMC5883L_CFGA_ODR_200HZ             (1 << 3) | (1 << 2)
#define QMC5883L_CFGA_MODE_STANDBY          (0 << 1) | (0 << 0)
#define QMC5883L_CFGA_MODE_CONTINUE         (0 << 1) | (1 << 0)
 
#define QMC5883L_CFGB_SOFT_RST              (1 << 7)
#define QMC5883L_CFGB_ROL_PNT               (1 << 6)
#define QMC5883L_CFGB_INT_ENB               (1 << 0)

#define QMC5883L_CFGA_VALUE_STANDBY             ( QMC5883L_CFGA_OSR_512        \
                                                | QMC5883L_CFGA_RNG_8G         \
                                                | QMC5883L_CFGA_ODR_200HZ      \
                                                | QMC5883L_CFGA_MODE_STANDBY )  
                                                /* OSR = 512;RNG = 8G;ODR=200Hz;MODE:待机模式 */
#define QMC5883L_CFGA_VALUE_CONTINUE            ( QMC5883L_CFGA_OSR_512        \
                                                | QMC5883L_CFGA_RNG_8G         \
                                                | QMC5883L_CFGA_ODR_200HZ      \
                                                | QMC5883L_CFGA_MODE_CONTINUE ) 
                                                /* OSR = 512;RNG = 8G;ODR=200Hz;MODE:连续模式 */
#define QMC5883L_CFGB_VALUE_REBOOT              ( QMC5883L_CFGB_SOFT_RST )     
 
#define QMC5883L_CFGC_VALUE                 0x40
#define QMC5883L_CFGD_VALUE                 0x01
#define QMC5883L_PERIORC_VALUE              0x01
 
#define QMC5883L_SENSITIVITY_2G             ( 12 )
#define QMC5883L_SENSITIVITY_8G             ( 3 )

#define QMC5883L_2G_SCALE                   (float)((2 * 2) / 65536.0)
#define QMC5883L_8G_SCALE                   (float)((2 * 8) / 65536.0)

enum QMC5883L_ORITENATION {
    QMC5883L_ORITENATION_NULL = 0x00,
    QMC5883L_ORITENATION_YAW_270, //pilot x -> sensor y, pilot y -> sensor -x
};

class qmc5883l : public i2cDevice
{
public:
    qmc5883l() = delete;
    qmc5883l(enum QMC5883L_ORITENATION _ori, uint32_t _id, ESAF::ebus_i2cmaster *i2c);
    ~qmc5883l() = default;

	uint8_t checkid();
	uint8_t init();
    void reset();

	void update();
	void readmag(int16_t *_mag);

public:
    
	void writeRegister(uint8_t regAddress, uint8_t data);
	void readRegister(uint8_t regAddress, uint8_t *buf, uint8_t len, int rwway);    

    uint32_t device_id;
    uint64_t timestamp_start_block;

    uint8_t orintation;

    uint8_t rawmag[6];
    int16_t mag[3];
    float mag_ga[3];
	ESAF::ebus_i2cmaster *_i2c;

    #ifdef uOSB_PUBLISH_QMC5883L
        uORB::uORBtopicsTypeMap<uORB::TOPIC_SENSOR_MAG>::type _pub_sensor_mag;
    #endif
};

#endif
