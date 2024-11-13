#include "board_config.h"

void error_handler()
{
	while (1) {}
}

void board_app_vector_init()
{
    SCB->VTOR = APP_LOAD_ADDRESS;
	
    HAL_RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    __set_BASEPRI(0);
    __set_FAULTMASK(0);
    __set_PRIMASK(0);
    __enable_irq();
}

void board_io_array_init()
{
	GPIO_InitTypeDef gpio_init;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
	
	// Red Led and Blue Led
	gpio_init.Pin = GPIO_nLED_RED_PIN | GPIO_nLED_BLUE_PIN;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIO_nLED_RED_PORT, &gpio_init);
	
	// power in detector
	gpio_init.Pin = GPIO_nPOWER_IN_A_PIN;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIO_nPOWER_IN_A_PORT, &gpio_init);
	gpio_init.Pin = GPIO_nPOWER_IN_B_PIN;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIO_nPOWER_IN_B_PORT, &gpio_init);
	gpio_init.Pin = GPIO_nPOWER_IN_C_PIN;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIO_nPOWER_IN_C_PORT, &gpio_init);
	
	// periph output ctrl and end
	gpio_init.Pin = GPIO_VDD_5V_PERIPH_nEN_PIN;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_VDD_5V_PERIPH_nEN_PORT, &gpio_init);
	gpio_init.Pin = GPIO_VDD_5V_PERIPH_nOC_PIN;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_VDD_5V_PERIPH_nOC_PORT, &gpio_init);
	
	// hipower output ctrl and end
	gpio_init.Pin = GPIO_VDD_5V_HIPOWER_nEN_PIN;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_VDD_5V_HIPOWER_nEN_PORT, &gpio_init);
	gpio_init.Pin = GPIO_VDD_5V_HIPOWER_nOC_PIN;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_VDD_5V_HIPOWER_nOC_PORT, &gpio_init);

	gpio_init.Pin = GPIO_VDD_3V3_SENSORS_EN_PIN;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_VDD_3V3_SENSORS_EN_PORT, &gpio_init);
	
	gpio_init.Pin = GPIO_OTGFS_VBUS_PIN;
	gpio_init.Mode = GPIO_MODE_INPUT;
	gpio_init.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIO_OTGFS_VBUS_PORT, &gpio_init);
}

void board_app_init()
{
	VDD_5V_PERIPH_EN(true);
	VDD_5V_HIPOWER_EN(true);

	BOARD_BLUE_LED(false);
	BOARD_RED_LED(false);
}

void board_red_led_toggle()
{
	int val = HAL_GPIO_ReadPin(GPIO_nLED_RED_PORT, GPIO_nLED_RED_PIN);
	HAL_GPIO_WritePin(GPIO_nLED_RED_PORT, GPIO_nLED_RED_PIN, !val);
}

void board_blue_led_toggle()
{
	int val = HAL_GPIO_ReadPin(GPIO_nLED_BLUE_PORT, GPIO_nLED_BLUE_PIN);
	HAL_GPIO_WritePin(GPIO_nLED_BLUE_PORT, GPIO_nLED_BLUE_PIN, !val);
}

void board_debug_delay()
{
	for(unsigned int i = 0; i < 5000 * 1000; i++);
	for(unsigned int i = 0; i < 5000 * 1000; i++);
	for(unsigned int i = 0; i < 5000 * 1000; i++);
}

void board_system_rcc_config()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable PWR_EXTERNAL_SOURCE_SUPPLY PWR_LDO_SUPPLY
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    * HSE: 16M -> 16M / PLLM * PLLN / PLLP
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 60;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    //RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        error_handler();
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

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        error_handler();
    }

    __HAL_RCC_CSI_ENABLE() ;
    __HAL_RCC_SYSCFG_CLK_ENABLE() ;

    HAL_EnableCompensationCell();
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();

    __HAL_RCC_BKPRAM_CLKAM_ENABLE();
    __HAL_RCC_D3SRAM1_CLKAM_ENABLE();
}
