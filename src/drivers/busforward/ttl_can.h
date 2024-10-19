
#ifndef __TTL_CAN_H_
#define __TTL_CAN_H_

#include "ebus_can.hpp"
#include "ebus_uart.hpp"
#include "econtainer_driver.hpp"
#include "CANFrame.hpp"
#include "serialize.hpp"
#include "__h7_pin_resource_global.h"
#include "h7_gpio.h"

#pragma pack(1)
struct TTL_CAN_Frame
{
	uint8_t _thead;
	uint8_t _tidyype;
	uint8_t _tdatatype;
	uint8_t _tdatalength;
	uint32_t _tid;
	uint8_t _tdata[8];
	uint8_t _ttail;
};
#pragma pack()

class ttl_can : public ESAF::ebus_can, public ESAF::edevbuffer<ESAF::CANFrame>
{
public:
	enum TTLCAN_MACRO {
		TTLCAN_FRAMEHEAD = 0xAA,
		TTLCAN_FRAMETAIL = 0x7A,
		TTLCAN_FRAMEMINLEN = 0x05,
		TTLCAN_STRANDARD_FRAME = 0x00,
		TTLCAN_EXTENDED_FRAME = 0x01,
	};

	ttl_can() = delete;
    ttl_can(size_t size, ESAF::Mutex *rm = NULL, ESAF::Mutex *wm = NULL);
    ttl_can(size_t read_size, size_t write_size, ESAF::Mutex *rm = NULL, ESAF::Mutex *wm = NULL);
	~ttl_can();

	void configttlbus(ESAF::ebus_uart *com);

    int init();

    int update();

    int read(void *pdata, unsigned int count);

    int read_through(void *pdata, unsigned int count, int rwway);

    int write(const void *pdata, unsigned int count);

    int write_through(const void *pdata, unsigned int count, int rwway);
	
    int query(unsigned int offset, void *pdata, unsigned int count);
	
    void flush(int rwstream);
	
    void eclear(int rwstream);
	
    unsigned int esize(int rwstream);
	
    int ioctl(unsigned int operation, unsigned int &arg);

public:
    void transmit();
	uint8_t receive();

	void devbuf_retrieve(uint16_t len);
protected:
	void sendframe(ESAF::CANFrame *msg);

protected:
    ESAF::ebus_uart *_com;
    uint8_t retrieve[50];
};



#endif

