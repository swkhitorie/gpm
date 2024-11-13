/**
 * low level driver for stm32h7 series, base on cubehal library
 * module external interrupt
*/

#include "lld_exirq.h"

lld_exirq_t *mcu_exit_list[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void lld_exit_init(lld_exirq_t *obj, GPIO_TypeDef *port, uint8_t pin, uint32_t mode, uint32_t pull)
{
    lld_gpio_init(&obj->it_pin, port, pin, mode, pull, GPIO_SPEED_FREQ_HIGH, 0);
    obj->it_line = (uint32_t)(0x01 << pin);
    
	IRQn_Type tmpIRQn[16] = {
		EXTI0_IRQn,			EXTI1_IRQn,			EXTI2_IRQn,			EXTI3_IRQn,      /* EXIT IRQ 0~3 */
		EXTI4_IRQn,			EXTI9_5_IRQn,		EXTI9_5_IRQn,		EXTI9_5_IRQn,    /* EXIT IRQ 4~7 */
		EXTI9_5_IRQn,		EXTI9_5_IRQn,		EXTI15_10_IRQn,		EXTI15_10_IRQn,  /* EXIT IRQ 8~11 */
		EXTI15_10_IRQn,		EXTI15_10_IRQn,		EXTI15_10_IRQn,		EXTI15_10_IRQn}; /* EXIT IRQ 12~15 */
	
	HAL_NVIC_SetPriority(tmpIRQn[pin], 2, 0);
	HAL_NVIC_EnableIRQ(tmpIRQn[pin]);
        
    mcu_exit_list[pin] = obj;
}

void lld_exit_irq(lld_exirq_t *obj)
{
	if(lld_gpio_get(&obj->it_pin) && obj->rising) {
        obj->rising();
    } else if (!lld_gpio_get(&obj->it_pin) && obj->falling) {
        obj->falling();
    }   
}
