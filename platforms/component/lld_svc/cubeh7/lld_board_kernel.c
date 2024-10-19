/**
 * low level driver for stm32h7 series, base on cubehal library
 * module kernel of board
*/
#include "lld_kernel.h"

void lld_kernel_board_cfgclk(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;
	
    #if (KERNEL_BOARD_SELECTION == KERNEL_BOARD_STM32H743xx)
        {
            // clock and power system for stm32h743xx
            MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
            __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
            while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

            RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
            RCC_OscInitStruct.HSEState = RCC_HSE_ON;
            RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
            RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
            RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
            RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;	
            RCC_OscInitStruct.PLL.PLLM = 5;
            RCC_OscInitStruct.PLL.PLLN = 160;
            RCC_OscInitStruct.PLL.PLLP = 2;
            RCC_OscInitStruct.PLL.PLLQ = 4;		
            RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
            RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;	
            ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);

            RCC_ClkInitStruct.ClockType=(RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | \
                                        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);
            RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
            RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
            RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
            RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
            RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
            RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;
            ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
            __HAL_RCC_CSI_ENABLE();
            __HAL_RCC_SYSCFG_CLK_ENABLE(); 
            HAL_EnableCompensationCell();
        }
    #elif (KERNEL_BOARD_SELECTION == KERNEL_BOARD_STM32H7B0xxQ)
        {
            // clock and power system for stm32h7b0xxq
            RCC->CKGAENR = 0xFFFFFFFF;
            HAL_PWREx_ConfigSupply(PWR_SMPS_1V8_SUPPLIES_LDO);
            __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
            while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
            RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
            RCC_OscInitStruct.HSEState = RCC_HSE_ON;
            RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
            RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
            RCC_OscInitStruct.PLL.PLLM = 13;
            RCC_OscInitStruct.PLL.PLLN = 280;
            RCC_OscInitStruct.PLL.PLLP = 2;
            RCC_OscInitStruct.PLL.PLLQ = 2;
            RCC_OscInitStruct.PLL.PLLR = 2;
            RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
            RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
            RCC_OscInitStruct.PLL.PLLFRACN = 0;
            HAL_RCC_OscConfig(&RCC_OscInitStruct);

            RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                          |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                          |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
            RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
            RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
            RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
            RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
            RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
            RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
            RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
            HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
        }
    #endif
}

void lld_kernel_board_cfgmpu(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;
    HAL_MPU_Disable();
    
    #if (KERNEL_BOARD_SELECTION == KERNEL_BOARD_STM32H743xx)
        {
            /* set AXI RAM Write through, read allocate¡ê?no write allocate */
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
        //	/* set SRAM1 + SRAM2 Write through, read allocate¡ê?no write allocate */
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
        //	/* set SRAM3 Write through, read allocate¡ê?no write allocate */
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
            /* set SRAM4 Write through, read allocate¡ê?no write allocate */
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
        }
    #elif (KERNEL_BOARD_SELECTION == KERNEL_BOARD_STM32H7B0xxQ)
        {
            MPU_InitStruct.Enable = MPU_REGION_ENABLE;
            MPU_InitStruct.Number = MPU_REGION_NUMBER0;
            MPU_InitStruct.BaseAddress = 0x0;
            MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
            MPU_InitStruct.SubRegionDisable = 0x87;
            MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
            MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
            MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
            MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
            MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
            MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
        }
    #endif
        
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}




