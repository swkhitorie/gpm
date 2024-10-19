#ifndef __LLD_KERNEL_H_
#define __LLD_KERNEL_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * low level driver for stm32h7 series, base on cubehal library
 * module kernel 
*/

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx.h"

/**
 * @brief board type selection
 * 1 --- stm32h743xx
 * 2 --- stm32h7B0xxQ
*/
#define KERNEL_BOARD_STM32H743xx  (1)
#define KERNEL_BOARD_STM32H7B0xxQ (2)

#if defined (STM32H7B3xx) || defined (STM32H7B3xxQ) || \
    defined (STM32H7B0xx) || defined (STM32H7B0xxQ)
#define KERNEL_BOARD_SELECTION    KERNEL_BOARD_STM32H7B0xxQ

#elif defined (STM32H743xx) || defined (STM32H753xx) || \
      defined (STM32H750xx)
#define KERNEL_BOARD_SELECTION    KERNEL_BOARD_STM32H743xx

#endif
    
/**
 * @brief global func to handle interrupt (except for NMI/HardFault)
*/
#define ENABLE_INT()        __set_PRIMASK(0)
#define DISABLE_INT()       __set_PRIMASK(1)

/**
 * @brief H7 memory allocation, address and size
*/
#define RAM_DTCM_ADDR       (0x20000000)
#define	RAM_AXI_D1_ADDR     (0x24000000)
#define	RAM_SRAM1_D2_ADDR   (0x30000000)
#define	RAM_SRAM2_D2_ADDR   (0x30020000)
#define	RAM_SRAM3_D2_ADDR   (0x30040000)
#define	RAM_SRAM4_D3_ADDR   (0x38000000)

#define	RAM_DTCM_LEN        (0x20000)			/* 128K */
#define	RAM_AXI_D1_LEN      (0x80000)			/* 512K */
#define	RAM_SRAM1_D2_LEN    (0x20000)			/* 128K */
#define	RAM_SRAM2_D2_LEN    (0x20000)			/* 128K */
#define	RAM_SRAM3_D2_LEN    (0x8000)			/* 128K */
#define	RAM_SRAM4_D3_LEN    (0x10000)			/* 64K */

typedef enum __lld_rwway {
    RWPOLL,
    RWIT,
    RWDMA
} lld_rwway;
    
void        lld_kernel_init(uint32_t init);
void        lld_kernel_board_cfgmpu();
void        lld_kernel_board_cfgclk();

void        lld_kernel_reboot();
void        lld_kernel_irq();
void        lld_kernel_delay_us(uint32_t n);
uint64_t    lld_kernel_get_time(uint32_t way);

#ifdef __cplusplus
}
#endif

#endif
