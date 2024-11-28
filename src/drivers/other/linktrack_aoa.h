#ifndef LINKTRACK_AOA_H_
#define LINKTRACK_AOA_H_

#include "ebus_uart.hpp"

#pragma pack(1)
typedef struct linktrack_aoa_pack {
    uint16_t len; 
    uint8_t nodeRole;  
	uint8_t nodeId;
    uint32_t local_time;
	uint32_t system_time;
    uint32_t nodeReserved;
    uint16_t voltage_org;
    uint8_t valid_node_quantity;
	uint8_t role;
	uint8_t id;
	uint8_t dis1;
    uint8_t dis2;
    uint8_t dis3;
	int16_t angle;
	uint8_t fp_rssi;
	uint8_t rx_rssi;
	int16_t reserved;
    uint8_t check;	
} trackaoa_pack_t;
#pragma pack()

class linktrack_aoa
{
public:
    enum MAGIC {
        LINKTRACK_FRAME_HEADER = 0x55,
        LINKTRACK_FUNC_MARK = 0x07,
        LINKTRACK_MIN_LEN = 3,
        
    };
    linktrack_aoa() = delete;
    linktrack_aoa(ESAF::ebus_uart *com);
    ~linktrack_aoa() = default;      
    
    int8_t update();

public: 
    uint8_t checksum(uint8_t *p, uint32_t len);

    void devbuf_retrieve(uint16_t len);

    trackaoa_pack_t _pack;

    float _distance;

    float _angle;
    float _lst_angle;

    float _localtime;
    float _lst_localtime;
    float _systemtime;
    float _voltage;
    float _rxRssi;
    float _fpRssi;

    bool _effective;

    uint8_t _retrieve[50];
    ESAF::ebus_uart *_com;
};

#endif
