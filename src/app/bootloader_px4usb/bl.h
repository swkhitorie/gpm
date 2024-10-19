#ifndef BL_H_
#define BL_H_

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "bl_flash.h"
#include "bl_kernel.h"
/**
    QGC  ----------------------------------------------------> PILOT
                    PROTO_GET_SYNC + PROTO_EOC
    QGC  <---------------------------------------------------- PILOT
                    PROTO_INSYNC + PROTO_OK
                    
    QGC  ----------------------------------------------------> PILOT
            PROTO_GET_DEVICE + PROTO_DEVICE_BL_REV(bootloader revision) + PROTO_EOC                
    QGC  <---------------------------------------------------- PILOT
                (uint32_t)4 + PROTO_INSYNC + PROTO_OK
                
    QGC  ----------------------------------------------------> PILOT
            PROTO_GET_DEVICE + PROTO_DEVICE_BOARD_ID(board ID) + PROTO_EOC   
    QGC  <---------------------------------------------------- PILOT
                (uint32_t)9 + PROTO_INSYNC + PROTO_OK    
                
    QGC  ----------------------------------------------------> PILOT
            PROTO_GET_DEVICE + PROTO_DEVICE_FW_SIZE(size of flashable area) + PROTO_EOC 
    QGC  <---------------------------------------------------- PILOT
          (uint32_t) (2,080,768) 00 C0 1F 00 + PROTO_INSYNC + PROTO_OK  
          
    QGC  ----------------------------------------------------> PILOT
            PROTO_CHIP_ERASE + PROTO_EOC   
            PILOT Start Erasing ---> 
    QGC  <---------------------------------------------------- PILOT
                PROTO_INSYNC + PROTO_OK          

    QGC  ----------------------------------------------------> PILOT
                    FILE
    QGC  <---------------------------------------------------- PILOT
                PROTO_INSYNC + PROTO_OK          

    QGC  ----------------------------------------------------> PILOT
                    PROTO_GET_CRC + PROTO_EOC
    QGC  <---------------------------------------------------- PILOT
                PROTO_INSYNC + PROTO_OK    

    QGC  ----------------------------------------------------> PILOT
                    PROTO_BOOT + PROTO_EOC
    QGC  <---------------------------------------------------- PILOT
                PROTO_INSYNC + PROTO_OK  

*/

struct boardinfo {
	uint32_t  board_type;
	uint32_t  board_rev;
	uint32_t  fw_size;
	uint32_t  systick_mhz;    /* systick input clock */
} __attribute__((packed));
extern struct boardinfo board_info;

void bt_init();
void bootloader();
void jump_to_app();
uint32_t bl_get_state();

#endif
