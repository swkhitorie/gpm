
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "fr_task.h"

extern void cdc_acm_print(uint8_t busid, const char *fmt, ...);
void NMI_Handler(void)
{
    board_red_led_toggle();
}

void HardFault_Handler(void)
{
  while (1)
  {
    board_red_led_toggle();
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
    board_red_led_toggle();
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
    board_red_led_toggle();
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
    board_red_led_toggle();
  }
}

void DebugMon_Handler(void)
{
}

// void PPP_IRQHandler(void)
// {
// }

// void SVC_Handler(void)
// {
// }

// void PendSV_Handler(void)
// {
// }

void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();	
    }
    HAL_IncTick();
}

