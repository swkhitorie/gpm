#include "FreeRTOS.h"
#include "fr_task.h"

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, 
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{

}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, 
    StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{

}

void vApplicationMallocFailedHook( void )
{
    extern void cdc_acm_print(uint8_t busid, const char *fmt, ...);
    cdc_acm_print(0, "malloc failed \r\n");
}