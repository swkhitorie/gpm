#ifndef BL_KERNEL_H_
#define BL_KERNEL_H_

#include "stm32h7xx.h"
#include "core_cm7.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#define ENABLE_INT()						__set_PRIMASK(0)
#define DISABLE_INT()						__set_PRIMASK(1)

#define		RAM_DTCM_ADDR					(0x20000000)
#define		RAM_AXI_D1_ADDR					(0x24000000)
#define		RAM_SRAM1_D2_ADDR				(0x30000000)
#define		RAM_SRAM2_D2_ADDR				(0x30020000)
#define		RAM_SRAM3_D2_ADDR				(0x30040000)
#define		RAM_SRAM4_D3_ADDR				(0x38000000)

#define		RAM_DTCM_LEN					(0x20000)			/* 128K */
#define		RAM_AXI_D1_LEN					(0x80000)			/* 512K */
#define		RAM_SRAM1_D2_LEN				(0x20000)			/* 128K */
#define		RAM_SRAM2_D2_LEN				(0x20000)			/* 128K */
#define		RAM_SRAM3_D2_LEN				(0x8000)			/* 128K */
#define		RAM_SRAM4_D3_LEN				(0x10000)			/* 64K */

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  
typedef const int16_t sc16;  
typedef const int8_t sc8;  

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  
typedef __I int16_t vsc16; 
typedef __I int8_t vsc8;   

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  
typedef const uint16_t uc16;  
typedef const uint8_t uc8; 

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  
typedef __I uint16_t vuc16; 
typedef __I uint8_t vuc8; 

uint32_t get_tick();
uint64_t get_microseconds_m1();
uint64_t get_microseconds_m2();
void reboot();
void start_up();

void cache_enable();
void cache_writethrough();
void cache_disable();
void kernelClock_config();
void mpu_config();

void delay_us(uint32_t n);

#endif
