
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "fr_task.h"

extern void cdc_acm_print(uint8_t busid, const char *fmt, ...);
void NMI_Handler(void)
{
cdc_acm_print(0, "e1 \r\n");
}

void HardFault_Handler(void)
{
  while (1)
  {cdc_acm_print(0, "e2 \r\n");
  }
}

void MemManage_Handler(void)
{
  while (1)
  {cdc_acm_print(0, "e13 \r\n");
  }
}

void BusFault_Handler(void)
{
  while (1)
  {cdc_acm_print(0, "e14 \r\n");
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {cdc_acm_print(0, "e5 \r\n");
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

