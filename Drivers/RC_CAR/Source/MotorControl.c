#include "MotorControl.h"
#include <string.h>  // For memset

GPIO_InitTypeDef Motor_gpio_structure;

// ADC handle for motor current sensing using ADC3
ADC_HandleTypeDef hadc3;  // For PC1 (MT_A_SENSE) - using ADC3 Channel 11
ADC_ChannelConfTypeDef sConfig = {0};
ADC_MultiModeTypeDef multimode = {0};

void MotorControl_Init(void)
{
    // Enable all required GPIO clocks
    MT_A_EN_GPIO_CLK_ENABLE();
    MT_A_IN1_GPIO_CLK_ENABLE();
    MT_A_IN2_GPIO_CLK_ENABLE();
    MT_A_SENSE_GPIO_CLK_ENABLE();
    MT_B_EN_GPIO_CLK_ENABLE();
    MT_B_IN1_GPIO_CLK_ENABLE();
    MT_B_IN2_GPIO_CLK_ENABLE();
    MT_B_SENSE_GPIO_CLK_ENABLE();
    
    // Initialize Motor A Enable Pin (PWM)
    Motor_gpio_structure.Pin = MT_A_EN_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_AF_PP;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    Motor_gpio_structure.Alternate = MT_A_EN_AF;
    HAL_GPIO_Init(MT_A_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor A Direction Control Pin 1
    Motor_gpio_structure.Pin = MT_A_IN1_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_OUTPUT_PP;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    Motor_gpio_structure.Alternate = MT_A_IN1_AF;
    HAL_GPIO_Init(MT_A_IN1_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor A Direction Control Pin 2
    Motor_gpio_structure.Pin = MT_A_IN2_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_OUTPUT_PP;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    Motor_gpio_structure.Alternate = MT_A_IN2_AF;
    HAL_GPIO_Init(MT_A_IN2_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor A Current Sense Pin (ADC)
    Motor_gpio_structure.Pin = MT_A_SENSE_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_ANALOG;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_LOW;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    // No alternate function for analog pins
    HAL_GPIO_Init(MT_A_SENSE_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor B Enable Pin (PWM)
    Motor_gpio_structure.Pin = MT_B_EN_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_AF_PP;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    Motor_gpio_structure.Alternate = MT_B_EN_AF;
    HAL_GPIO_Init(MT_B_EN_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor B Direction Control Pin 1
    Motor_gpio_structure.Pin = MT_B_IN1_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_OUTPUT_PP;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    Motor_gpio_structure.Alternate = MT_B_IN1_AF;
    HAL_GPIO_Init(MT_B_IN1_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor B Direction Control Pin 2
    Motor_gpio_structure.Pin = MT_B_IN2_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_OUTPUT_PP;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    Motor_gpio_structure.Alternate = MT_B_IN2_AF;
    HAL_GPIO_Init(MT_B_IN2_GPIO_PORT, &Motor_gpio_structure);
    
    // Initialize Motor B Current Sense Pin (ADC)
    Motor_gpio_structure.Pin = MT_B_SENSE_PIN;
    Motor_gpio_structure.Mode = GPIO_MODE_ANALOG;
    Motor_gpio_structure.Speed = GPIO_SPEED_FREQ_LOW;
    Motor_gpio_structure.Pull = GPIO_NOPULL;
    // No alternate function for analog pins
    HAL_GPIO_Init(MT_B_SENSE_GPIO_PORT, &Motor_gpio_structure);
    
    // Set initial states - Motors disabled
    HAL_GPIO_WritePin(MT_A_IN1_GPIO_PORT, MT_A_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT_A_IN2_GPIO_PORT, MT_A_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT_B_IN1_GPIO_PORT, MT_B_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT_B_IN2_GPIO_PORT, MT_B_IN2_PIN, GPIO_PIN_RESET);
}

void MotorControl_ADC_Reset(void)
{
    uint8_t debug_msg[] = "MotorControl_ADC_Reset: Starting ADC3 hardware reset...\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], debug_msg, strlen((char*)debug_msg), HAL_MAX_DELAY);
    
    // Force ADC3 reset
    __HAL_RCC_ADC3_FORCE_RESET();
    HAL_Delay(5);
    __HAL_RCC_ADC3_RELEASE_RESET();
    HAL_Delay(5);
    
    // Clear any previous state
    memset(&hadc3, 0, sizeof(hadc3));
    
    uint8_t reset_complete[] = "MotorControl_ADC_Reset: ADC3 hardware reset complete\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], reset_complete, strlen((char*)reset_complete), HAL_MAX_DELAY);
}

void MotorControl_ADC_Init(void)
{
    uint8_t debug_start[] = "MotorControl_ADC_Init: Starting ADC3 initialization...\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], debug_start, strlen((char*)debug_start), HAL_MAX_DELAY);
    
    // Perform complete ADC3 reset first
    //MotorControl_ADC_Reset();
    
    // Enable ADC3 clock
    __HAL_RCC_ADC3_CLK_ENABLE();
    HAL_Delay(10);  // Allow clock to stabilize
    
    // Configure ADC3 for PC1 (MT_A_SENSE) - Channel 11
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;  // Slower clock for stability
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc3.Init.LowPowerAutoWait = DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc3.Init.OversamplingMode = DISABLE;
    
    uint8_t debug_config[] = "MotorControl_ADC_Init: Configuring ADC3...\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], debug_config, strlen((char*)debug_config), HAL_MAX_DELAY);
    
    HAL_StatusTypeDef adc3_status = HAL_ADC_Init(&hadc3);
    if (adc3_status != HAL_OK)
    {
        uint8_t error_msg[60];
        sprintf((char*)error_msg, "ADC3 Init Error: %d, State: 0x%lX\r\n", 
                adc3_status, (unsigned long)hadc3.State);
        HAL_UART_Transmit(&hcom_uart[COM1], error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);
        Error_Handler();
    }
    
      /** Configure the ADC multi-mode
  */
    // multimode.Mode = ADC_MODE_INDEPENDENT;
    // if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
    // {
    //     Error_Handler();
    // }

    // uint8_t debug_calib[] = "MotorControl_ADC_Init: Starting ADC3 calibration...\r\n";
    // HAL_UART_Transmit(&hcom_uart[COM1], debug_calib, strlen((char*)debug_calib), HAL_MAX_DELAY);
    
    // // Calibrate ADC3 for better accuracy
    // HAL_StatusTypeDef cal_status = HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // if (cal_status != HAL_OK)
    // {
    //     uint8_t error_msg[50];
    //     sprintf((char*)error_msg, "ADC3 Calibration Error: %d\r\n", cal_status);
    //     HAL_UART_Transmit(&hcom_uart[COM1], error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);
    // }
    // else
    // {
    //     uint8_t calib_ok[] = "MotorControl_ADC_Init: ADC3 calibration successful\r\n";
    //     HAL_UART_Transmit(&hcom_uart[COM1], calib_ok, strlen((char*)calib_ok), HAL_MAX_DELAY);
    // }
    
    // Configure ADC3 Channel 11 for PC1 (Motor A Current Sense)
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;  // PC1 is ADC3 Channel 11
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;  // Longer sampling for stability
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    
    uint8_t debug_channel[] = "MotorControl_ADC_Init: Configuring ADC3 Channel 11...\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], debug_channel, strlen((char*)debug_channel), HAL_MAX_DELAY);
    
    HAL_StatusTypeDef adc3_ch_status = HAL_ADC_ConfigChannel(&hadc3, &sConfig);
    if (adc3_ch_status != HAL_OK)
    {
        uint8_t error_msg[60];
        sprintf((char*)error_msg, "ADC3 Ch Config Error: %d, State: 0x%lX\r\n", adc3_ch_status, (unsigned long)hadc3.State);
        HAL_UART_Transmit(&hcom_uart[COM1], error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);
        Error_Handler();
    }
    
    // Wait for ADC voltage regulator to stabilize
    HAL_Delay(50);  // Initial delay for regulator
    
    uint8_t debug_enable[] = "MotorControl_ADC_Init: Enabling ADC3...\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], debug_enable, strlen((char*)debug_enable), HAL_MAX_DELAY);
    

    HAL_Delay(100);  // Additional delay after enabling

    uint8_t debug_msg[80];
    sprintf((char*)debug_msg, "ADC3 initialization complete - State: 0x%lX, ready for conversions!\r\n", (unsigned long)hadc3.State);
    HAL_UART_Transmit(&hcom_uart[COM1], debug_msg, strlen((char*)debug_msg), HAL_MAX_DELAY);

    
}

uint16_t MotorControl_ReadCurrentA(void)
{
    uint16_t adcValue = 0;
    static uint8_t error_count = 0;
    const uint8_t MAX_ERRORS = 3;
    
    uint8_t debug_msg[80];
    sprintf((char*)debug_msg, "MotorControl_ReadCurrentA: ADC3 state before start = 0x%lX\r\n", (unsigned long)hadc3.State);
    HAL_UART_Transmit(&hcom_uart[COM1], debug_msg, strlen((char*)debug_msg), HAL_MAX_DELAY);
    
    // Check if ADC is in error state and try to recover
    if (hadc3.State & HAL_ADC_STATE_ERROR_INTERNAL)
    {
        uint8_t error_msg[90];
        sprintf((char*)error_msg, "MotorControl_ReadCurrentA: ADC3 in ERROR_INTERNAL state (0x%lX), attempting recovery...\r\n", 
                (unsigned long)hadc3.State);
        HAL_UART_Transmit(&hcom_uart[COM1], error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);
        
        error_count++;
        if (error_count >= MAX_ERRORS)
        {
            uint8_t reset_msg[] = "MotorControl_ReadCurrentA: Max errors reached, performing full ADC3 reset...\r\n";
            HAL_UART_Transmit(&hcom_uart[COM1], reset_msg, strlen((char*)reset_msg), HAL_MAX_DELAY);
            
            MotorControl_ADC_Init();  // Full re-initialization
            error_count = 0;
            return 0;  // Return early after reset
        }
        else
        {
            // Try to clear the error by disabling and re-enabling ADC
            uint8_t clear_error[] = "MotorControl_ReadCurrentA: Clearing ADC3 error state...\r\n";
            HAL_UART_Transmit(&hcom_uart[COM1], clear_error, strlen((char*)clear_error), HAL_MAX_DELAY);
            
            ADC_Disable(&hadc3);
            HAL_Delay(10);
            ADC_Enable(&hadc3);
            HAL_Delay(10);
        }
    }
    
    // Ensure ADC is ready before starting
    if (hadc3.State != HAL_ADC_STATE_READY)
    {
        uint8_t debug_stop[] = "MotorControl_ReadCurrentA: ADC3 not ready, stopping first...\r\n";
        HAL_UART_Transmit(&hcom_uart[COM1], debug_stop, strlen((char*)debug_stop), HAL_MAX_DELAY);
        
        HAL_ADC_Stop(&hadc3);
        HAL_Delay(10);
        
        sprintf((char*)debug_msg, "MotorControl_ReadCurrentA: ADC3 state after stop = 0x%lX\r\n", 
                (unsigned long)hadc3.State);
        HAL_UART_Transmit(&hcom_uart[COM1], debug_msg, strlen((char*)debug_msg), HAL_MAX_DELAY);
    }
    
    // Start ADC conversion
    HAL_StatusTypeDef start_status = HAL_ADC_Start(&hadc3);
    HAL_Delay(500);  // Longer delay for stability
    
    if (start_status == HAL_OK)
    {
        uint8_t debug_msg2[] = "MotorControl_ReadCurrentA: ADC3 started successfully, polling...\r\n";
        HAL_UART_Transmit(&hcom_uart[COM1], debug_msg2, strlen((char*)debug_msg2), HAL_MAX_DELAY);
        
        // Wait for conversion to complete with timeout
        HAL_StatusTypeDef poll_status = HAL_ADC_PollForConversion(&hadc3, 1000);  // Longer timeout
        if (poll_status == HAL_OK)
        {
            // Get the converted value
            adcValue = HAL_ADC_GetValue(&hadc3);
            error_count = 0;  // Reset error count on successful read
            
            uint8_t debug_msg3[60];
            sprintf((char*)debug_msg3, "MotorControl_ReadCurrentA: Successful read, Value = %d\r\n", adcValue);
            HAL_UART_Transmit(&hcom_uart[COM1], debug_msg3, strlen((char*)debug_msg3), HAL_MAX_DELAY);
        }
        else
        {
            error_count++;
            uint8_t error_msg[70];
            sprintf((char*)error_msg, "MotorControl_ReadCurrentA: Poll failed, status = %d, errors = %d\r\n", 
                    poll_status, error_count);
            HAL_UART_Transmit(&hcom_uart[COM1], error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);
        }
        
        // Stop ADC properly
        HAL_ADC_Stop(&hadc3);
    }
    else
    {
        error_count++;
        uint8_t error_msg[80];
        sprintf((char*)error_msg, "MotorControl_ReadCurrentA: Start failed, status = %d, state = 0x%lX, errors = %d\r\n", 
                start_status, (unsigned long)hadc3.State, error_count);
        HAL_UART_Transmit(&hcom_uart[COM1], error_msg, strlen((char*)error_msg), HAL_MAX_DELAY);
        
        // Try simple reset if ADC3 is stuck
        if (hadc3.State != HAL_ADC_STATE_READY)
        {
            uint8_t reset_msg[] = "MotorControl_ReadCurrentA: Attempting soft reset...\r\n";
            HAL_UART_Transmit(&hcom_uart[COM1], reset_msg, strlen((char*)reset_msg), HAL_MAX_DELAY);
            
            HAL_ADC_Stop(&hadc3);
            HAL_Delay(20);
        }
    }
    
    uint8_t debug_end[60];
    sprintf((char*)debug_end, "MotorControl_ReadCurrentA: Complete, returning %d\r\n", adcValue);
    HAL_UART_Transmit(&hcom_uart[COM1], debug_end, strlen((char*)debug_end), HAL_MAX_DELAY);
    
    return adcValue;
}