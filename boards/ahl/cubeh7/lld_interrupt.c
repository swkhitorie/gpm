/**
 * low level driver for stm32h7 series, base on cubehal library
 * all core and periph interrupt
*/

#include "lld_kernel.h"
#include "lld_exirq.h"
#include "lld_uart.h"
#include "lld_i2c.h"
#include "lld_can.h"
#include "lld_timer.h"
#include "platforms_common.h"

void SysTick_Handler(void)
{
    lld_kernel_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t idx = 0;
    
    if (huart->Instance == USART1)		idx = 0;
    else if (huart->Instance == USART2)	idx = 1;
    else if (huart->Instance == USART3)	idx = 2;
    else if (huart->Instance == UART4)	idx = 3;
    else if (huart->Instance == UART5)	idx = 4;
    else if (huart->Instance == USART6)	idx = 5;
    else if (huart->Instance == UART7)	idx = 6;
    else if (huart->Instance == UART8)	idx = 7;

    if (mcu_uart_list[idx]) {
        lld_uart_irq_txdma(mcu_uart_list[idx]);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t idx = 0;
    
    if (huart->Instance == USART1)		idx = 0;
    else if (huart->Instance == USART2)	idx = 1;
    else if (huart->Instance == USART3)	idx = 2;
    else if (huart->Instance == UART4)	idx = 3;
    else if (huart->Instance == UART5)	idx = 4;
    else if (huart->Instance == USART6)	idx = 5;
    else if (huart->Instance == UART7)	idx = 6;
    else if (huart->Instance == UART8)	idx = 7;

    if (mcu_uart_list[idx]) {
        lld_uart_irq_rxdma(mcu_uart_list[idx]);
    }
}
/**
 * @brief UARTx_IRQHandler
*/
/** UART1 IRQ */
void USART1_IRQHandler(void)
{
    if(mcu_uart_list[0])
        lld_uart_irq(mcu_uart_list[0]);
}

/** UART2 IRQ */
void USART2_IRQHandler(void)
{	
    if(mcu_uart_list[1])	
        lld_uart_irq(mcu_uart_list[1]);
}

/** UART3 IRQ */
void USART3_IRQHandler(void)
{	
    if(mcu_uart_list[2])	
        lld_uart_irq(mcu_uart_list[2]);
}

/** UART4 IRQ */
void UART4_IRQHandler(void)
{	
    if(mcu_uart_list[3])	
        lld_uart_irq(mcu_uart_list[3]);
}

/** UART5 IRQ */
void UART5_IRQHandler(void)
{
    if(mcu_uart_list[4])	
        lld_uart_irq(mcu_uart_list[4]);
}

/** UART6 IRQ */
void USART6_IRQHandler(void)
{
    if(mcu_uart_list[5])	
        lld_uart_irq(mcu_uart_list[5]);
}

/** UART7 IRQ */
void UART7_IRQHandler(void)
{
    if(mcu_uart_list[6])	
        lld_uart_irq(mcu_uart_list[6]);
}

/** UART8 IRQ */
void UART8_IRQHandler(void)
{
    if(mcu_uart_list[7])	
        lld_uart_irq(mcu_uart_list[7]);
}


/**
 * @brief DMA_IRQHandler
*/

/** DMA1 Stream0  IRQ */
void DMA1_Stream0_IRQHandler(void)
{  
    if(mcu_uart_list[0])	
        HAL_DMA_IRQHandler(mcu_uart_list[0]->huart.hdmatx);
}

/** DMA1 Stream1  IRQ */
void DMA1_Stream1_IRQHandler(void)
{  
    if(mcu_uart_list[1])	
        HAL_DMA_IRQHandler(mcu_uart_list[1]->huart.hdmatx);
}

/** DMA1 Stream2  IRQ */
void DMA1_Stream2_IRQHandler(void)
{  
    if(mcu_uart_list[2])	
        HAL_DMA_IRQHandler(mcu_uart_list[2]->huart.hdmatx);
}

/** DMA1 Stream3  IRQ */
void DMA1_Stream3_IRQHandler(void)
{  
    if(mcu_uart_list[3])	
        HAL_DMA_IRQHandler(mcu_uart_list[3]->huart.hdmatx);
}

/** DMA1 Stream4  IRQ */
void DMA1_Stream4_IRQHandler(void)
{  
    if(mcu_uart_list[4])	
        HAL_DMA_IRQHandler(mcu_uart_list[4]->huart.hdmatx);
}

/** DMA1 Stream5  IRQ */
void DMA1_Stream5_IRQHandler(void)
{  
    if(mcu_uart_list[5])	
        HAL_DMA_IRQHandler(mcu_uart_list[5]->huart.hdmatx);
}

/** DMA1 Stream6  IRQ */
void DMA1_Stream6_IRQHandler(void)
{  
    if(mcu_uart_list[6])	
        HAL_DMA_IRQHandler(mcu_uart_list[6]->huart.hdmatx);
}

/** DMA1 Stream7  IRQ */
void DMA1_Stream7_IRQHandler(void)
{  
    if(mcu_uart_list[7])	
        HAL_DMA_IRQHandler(mcu_uart_list[7]->huart.hdmatx);
}


/** DMA2 Stream0  IRQ */
void DMA2_Stream0_IRQHandler(void)
{  
    if(mcu_uart_list[0])	
        HAL_DMA_IRQHandler(mcu_uart_list[0]->huart.hdmarx);
}

/** DMA2 Stream1  IRQ */
void DMA2_Stream1_IRQHandler(void)
{  
    if(mcu_uart_list[1])	
        HAL_DMA_IRQHandler(mcu_uart_list[1]->huart.hdmarx);
}

/** DMA2 Stream2  IRQ */
void DMA2_Stream2_IRQHandler(void)
{  
    if(mcu_uart_list[2])	
        HAL_DMA_IRQHandler(mcu_uart_list[2]->huart.hdmarx);
}

/** DMA2 Stream3  IRQ */
void DMA2_Stream3_IRQHandler(void)
{  
    if(mcu_uart_list[3])	
        HAL_DMA_IRQHandler(mcu_uart_list[3]->huart.hdmarx);	
}

/** DMA2 Stream4  IRQ */
void DMA2_Stream4_IRQHandler(void)
{  
    if(mcu_uart_list[4])	
        HAL_DMA_IRQHandler(mcu_uart_list[4]->huart.hdmarx);	
}

/** DMA2 Stream5  IRQ */
void DMA2_Stream5_IRQHandler(void)
{  
    if(mcu_uart_list[5])	
        HAL_DMA_IRQHandler(mcu_uart_list[5]->huart.hdmarx);	
}

/** DMA2 Stream6  IRQ */
void DMA2_Stream6_IRQHandler(void)
{  
    if(mcu_uart_list[6])	
        HAL_DMA_IRQHandler(mcu_uart_list[6]->huart.hdmarx);	
}

/** DMA2 Stream7  IRQ */
void DMA2_Stream7_IRQHandler(void)
{  
    if(mcu_uart_list[7])	
        HAL_DMA_IRQHandler(mcu_uart_list[7]->huart.hdmarx);	
}

/**
 * @brief 	External interrupt callback function
 *			It will be called by EXTIx_x_IRQHandler
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case GPIO_PIN_0:  lld_exit_irq(mcu_exit_list[0]);  break;
    case GPIO_PIN_1:  lld_exit_irq(mcu_exit_list[1]);  break;
    case GPIO_PIN_2:  lld_exit_irq(mcu_exit_list[2]);  break;
    case GPIO_PIN_3:  lld_exit_irq(mcu_exit_list[3]);  break;
    case GPIO_PIN_4:  lld_exit_irq(mcu_exit_list[4]);  break;
    case GPIO_PIN_5:  lld_exit_irq(mcu_exit_list[5]);  break;
    case GPIO_PIN_6:  lld_exit_irq(mcu_exit_list[6]);  break;
    case GPIO_PIN_7:  lld_exit_irq(mcu_exit_list[7]);  break;
    case GPIO_PIN_8:  lld_exit_irq(mcu_exit_list[8]);  break;
    case GPIO_PIN_9:  lld_exit_irq(mcu_exit_list[9]);  break;
    case GPIO_PIN_10:  lld_exit_irq(mcu_exit_list[10]);  break;
    case GPIO_PIN_11:  lld_exit_irq(mcu_exit_list[11]);  break;
    case GPIO_PIN_12:  lld_exit_irq(mcu_exit_list[12]);  break;
    case GPIO_PIN_13:  lld_exit_irq(mcu_exit_list[13]);  break;
    case GPIO_PIN_14:  lld_exit_irq(mcu_exit_list[14]);  break;
    case GPIO_PIN_15:  lld_exit_irq(mcu_exit_list[15]);  break;
    }
}
/** EXTI0  IRQ */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); 
}

/** EXTI1  IRQ */
void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/** EXTI2  IRQ */
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/** EXTI3  IRQ */
void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/** EXTI4  IRQ */
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); 
}

/** EXTI5~9  IRQ */
void EXTI9_5_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5); 
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

/** EXTI10~15  IRQ */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10); 
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint8_t idx = 0;
    if (hi2c->Instance == I2C1)		idx = 0;
    else if (hi2c->Instance == I2C2)    idx = 1;
    else if (hi2c->Instance == I2C3)	idx = 2;
    else if (hi2c->Instance == I2C4)	idx = 3;
    if (mcu_i2c_list[idx]) {
        lld_i2c_irq_cmdcomplete(mcu_i2c_list[idx]);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint8_t idx = 0;
    if (hi2c->Instance == I2C1)		idx = 0;
    else if (hi2c->Instance == I2C2)    idx = 1;
    else if (hi2c->Instance == I2C3)	idx = 2;
    else if (hi2c->Instance == I2C4)	idx = 3;
    if (mcu_i2c_list[idx]) {
        lld_i2c_irq_cmdcomplete(mcu_i2c_list[idx]);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint8_t idx = 0;
    if (hi2c->Instance == I2C1)		idx = 0;
    else if (hi2c->Instance == I2C2)    idx = 1;
    else if (hi2c->Instance == I2C3)	idx = 2;
    else if (hi2c->Instance == I2C4)	idx = 3;
    if (mcu_i2c_list[idx]) {
        lld_i2c_irq_cmdcomplete(mcu_i2c_list[idx]);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
    if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF) {
    }
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&mcu_i2c_list[0]->hi2c);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&mcu_i2c_list[0]->hi2c);
}

void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&mcu_i2c_list[1]->hi2c);
}

void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&mcu_i2c_list[1]->hi2c);
}

void I2C3_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&mcu_i2c_list[2]->hi2c);
}

void I2C3_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&mcu_i2c_list[2]->hi2c);
}

void I2C4_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&mcu_i2c_list[3]->hi2c);
}

void I2C4_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&mcu_i2c_list[3]->hi2c);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (hfdcan->Instance == FDCAN1)
            lld_can_rx_irq(mcu_can_list[0], FDCAN_RX_FIFO0);
        else if (hfdcan->Instance == FDCAN2)
            lld_can_rx_irq(mcu_can_list[1], FDCAN_RX_FIFO0);
            
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        if (hfdcan->Instance == FDCAN1)
            lld_can_rx_irq(mcu_can_list[0], FDCAN_RX_FIFO1);
        else if (hfdcan->Instance == FDCAN2)
            lld_can_rx_irq(mcu_can_list[1], FDCAN_RX_FIFO1);
        
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
}
/**
 * @brief CAN_IRQHandler
*/
/** CAN1 Rx FIFO0 IRQ */
void FDCAN1_IT0_IRQHandler(void)
{
    if (mcu_can_list[0]) 
        HAL_FDCAN_IRQHandler(&(mcu_can_list[0]->hcan));  
}

/** CAN1 Rx FIFO1 IRQ */
void FDCAN1_IT1_IRQHandler(void)
{
    if (mcu_can_list[0]) 
        HAL_FDCAN_IRQHandler(&(mcu_can_list[0]->hcan));   
}

/** CAN2 Rx FIFO0 IRQ */
void FDCAN2_IT0_IRQHandler(void)
{
    if (mcu_can_list[1]) 
        HAL_FDCAN_IRQHandler(&(mcu_can_list[1]->hcan));
}

/** CAN2 Rx FIFO1 IRQ */
void FDCAN2_IT1_IRQHandler(void)
{
    if (mcu_can_list[1]) 
        HAL_FDCAN_IRQHandler(&(mcu_can_list[1]->hcan)); 
}



/**
 * @brief 	Timer Update IRQ callback Function (when coutner overflow)
 *			it will be called by HAL_TIM_IRQHandler
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)			lld_timer_irq(mcu_timer_list[0]);
    else if (htim->Instance == TIM2)	lld_timer_irq(mcu_timer_list[1]);
    else if (htim->Instance == TIM3)	lld_timer_irq(mcu_timer_list[2]);
    else if (htim->Instance == TIM4)	lld_timer_irq(mcu_timer_list[3]);
    else if (htim->Instance == TIM5)	lld_timer_irq(mcu_timer_list[4]);
    else if (htim->Instance == TIM6)	lld_timer_irq(mcu_timer_list[5]);
    else if (htim->Instance == TIM7)	lld_timer_irq(mcu_timer_list[6]);
    else if (htim->Instance == TIM8)	lld_timer_irq(mcu_timer_list[7]);
}

/**
 * @brief 	Timer InputCapture IRQ callback Function
 *			it will be called by HAL_TIM_IRQHandler
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)			lld_timer_irq(mcu_timer_list[0]);
    else if (htim->Instance == TIM2)	lld_timer_irq(mcu_timer_list[1]);
    else if (htim->Instance == TIM3)	lld_timer_irq(mcu_timer_list[2]);
    else if (htim->Instance == TIM4)	lld_timer_irq(mcu_timer_list[3]);
    else if (htim->Instance == TIM5)	lld_timer_irq(mcu_timer_list[4]);
    else if (htim->Instance == TIM6)	lld_timer_irq(mcu_timer_list[5]);
    else if (htim->Instance == TIM7)	lld_timer_irq(mcu_timer_list[6]);
    else if (htim->Instance == TIM8)	lld_timer_irq(mcu_timer_list[7]);
}

/** TIM1 Up  IRQ */
void TIM1_UP_IRQHandler(void)
{ 
    if (mcu_timer_list[0]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[0]->htim));
}

/** TIM1 CC  IRQ */
void TIM1_CC_IRQHandler(void)
{ 
    if (mcu_timer_list[0]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[0]->htim));
}

/** TIM2 IRQ */
void TIM2_IRQHandler(void)
{ 
    if (mcu_timer_list[1]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[1]->htim));
}

/** TIM3 IRQ */
void TIM3_IRQHandler(void)
{ 
    if (mcu_timer_list[2]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[2]->htim));
}

/** TIM4 IRQ */
void TIM4_IRQHandler(void)
{ 
    if (mcu_timer_list[3]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[3]->htim));
}

/** TIM5 IRQ */
void TIM5_IRQHandler(void) 
{ 
    if (mcu_timer_list[4]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[4]->htim));
}

/** TIM6 DAC IRQ */
void TIM6_DAC_IRQHandler(void)
{ 
    if (mcu_timer_list[5]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[5]->htim));
}

/** TIM7 IRQ */
void TIM7_IRQHandler(void)
{ 
    if (mcu_timer_list[6]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[6]->htim));
}

/** TIM8 UP IRQ */
void TIM8_UP_TIM13_IRQHandler(void)
{ 
    if (mcu_timer_list[7]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[7]->htim));
}

/** TIM8 CC IRQ */
void TIM8_CC_IRQHandler(void)
{ 
    if (mcu_timer_list[7]) 
        HAL_TIM_IRQHandler(&(mcu_timer_list[7]->htim));
}




