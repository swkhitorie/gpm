#ifndef BL_BOARD_H_
#define BL_BOARD_H_

#include "bl_kernel.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

extern UART_HandleTypeDef huart1;
void board_usart_init();
void board_usart_deinit();
extern void mprint(const char *fmt, ...);

#endif
