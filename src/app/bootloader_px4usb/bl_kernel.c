#include "bl_kernel.h"

uint32_t get_tick() { return HAL_GetTick(); }
uint64_t get_microseconds_m1()
{
    uint64_t m0 = (uint64_t)HAL_GetTick();
    __IO uint64_t u0 = (uint64_t)SysTick->VAL;
    
    uint64_t m1 = (uint64_t)HAL_GetTick();
    __IO uint64_t u1 = (uint64_t)SysTick->VAL;
    
    const uint64_t tms = (uint64_t)(SysTick->LOAD + 1);
    
    if (m1 != m0) {
        return (uint64_t)((m1 * 1000 + ((tms - u1) * 1000) / tms));
    } else {
        return (uint64_t)((m0 * 1000 + ((tms - u0) * 1000) / tms));
    }
}

uint64_t get_microseconds_m2()
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t m = HAL_GetTick();
    __IO uint32_t v = SysTick->VAL;
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
    return (m * 1000 + ((tms - v) * 1000) / tms);
}

void reboot() { NVIC_SystemReset();}

void start_up()
{
    HAL_Init();
    kernelClock_config();
    for (int i = 0; i < 1000; i++);
}

void cache_enable()
{
	SCB_EnableICache();
	SCB_EnableDCache();
}
void cache_disable()
{
	SCB_DisableICache();
	SCB_DisableDCache();
}
void cache_writethrough()
{
	SCB->CACR |= 1 << 2;
}

void kernelClock_config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {

  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {

  }
}


void mpu_config()
{
	MPU_Region_InitTypeDef MPU_InitStruct;
	
	HAL_MPU_Disable();

	/* set AXI RAM Write through, read allocate¨²?no write allocate */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = RAM_AXI_D1_ADDR;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

//	/* set SRAM1 + SRAM2 Write through, read allocate¨²?no write allocate */
//	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
//	MPU_InitStruct.BaseAddress      = RAM_SRAM1_D2_ADDR;
//	MPU_InitStruct.Size             = MPU_REGION_SIZE_256KB;
//	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
//	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
//	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
//	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
//	MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
//	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
//	MPU_InitStruct.SubRegionDisable = 0x00;
//	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

//	HAL_MPU_ConfigRegion(&MPU_InitStruct);

//	/* set SRAM3 Write through, read allocate¨²?no write allocate */
//	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
//	MPU_InitStruct.BaseAddress      = RAM_SRAM3_D2_ADDR;
//	MPU_InitStruct.Size             = MPU_REGION_SIZE_32KB;
//	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
//	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
//	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
//	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
//	MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
//	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
//	MPU_InitStruct.SubRegionDisable = 0x00;
//	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

//	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	
	/* set SRAM4 Write through, read allocate¨²?no write allocate */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = RAM_SRAM4_D3_ADDR;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;	
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


void delay_us(uint32_t n)
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


void SysTick_Handler(void)
{
    HAL_IncTick();
}	 

void HAL_Delay(uint32_t Delay)
{
    delay_us(Delay * 1000);
}
