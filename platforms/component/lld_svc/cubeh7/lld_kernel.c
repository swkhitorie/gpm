/**
 * low level driver for stm32h7 series, base on cubehal library
 * module kernel 
*/
#include "lld_kernel.h"

__weak void lld_kernel_board_cfgmpu() {}
__weak void lld_kernel_board_cfgclk() {}
    
void lld_kernel_init(uint32_t init)
{
    lld_kernel_board_cfgmpu();
	SCB_EnableICache();
	SCB_EnableDCache();
    HAL_Init();
    lld_kernel_board_cfgclk();
    __HAL_RCC_SYSCFG_CLK_ENABLE(); 
    SysTick_Config(SystemCoreClock / 1000);
    for (int i = 0; i < 1000; i++);
}

void lld_kernel_reboot()
{
    NVIC_SystemReset();
}

void lld_kernel_irq()
{
    HAL_IncTick();
}

volatile uint64_t abs_time = 0;
uint64_t lld_kernel_get_time(uint32_t way)
{
    abs_time = 0;
    
    if (way == 0) {
        uint64_t m0 = HAL_GetTick();
        volatile uint64_t u0 = SysTick->VAL;
        const uint64_t tms = SysTick->LOAD + 1;
        abs_time = (m0 * 1000 + ((tms - u0) * 1000) / tms);
    } else {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint32_t m = HAL_GetTick();
        volatile uint32_t v = SysTick->VAL;
        // If an overflow happened since we disabled irqs, it cannot have been
        // processed yet, so increment m and reload VAL to ensure we get the
        // post-overflow value.
        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) {
            ++m;
            v = SysTick->VAL;
        }
        // Restore irq status
        __set_PRIMASK(primask);
        const uint32_t tms = SysTick->LOAD + 1;
        abs_time = (m * 1000 + ((tms - v) * 1000) / tms);
    }
    return abs_time;
}

void lld_kernel_delay_us(uint32_t n)
{
	uint32_t ticks;
	uint32_t told;
	uint32_t tnow;
	uint32_t tcnt = 0;
	uint32_t reload;
	   
	reload = SysTick->LOAD;                
	ticks = n * (SystemCoreClock / 1000000);
	
	tcnt = 0;
	told = SysTick->VAL;

	while (1) {
		tnow = SysTick->VAL;    
		if (tnow != told) {    
			if (tnow < told) {
				tcnt += told - tnow;    
			}else {
				tcnt += reload - tnow + told;    
			}        
			told = tnow;
			if (tcnt >= ticks) {
				break;
			}
		}  
	}
}
