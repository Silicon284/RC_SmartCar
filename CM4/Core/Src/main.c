/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Ultrasonic_Sensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to test UART loopback (connect PD5 to PD6 temporarily)
// void test_uart_loopback(void)
// {
//   uint8_t test_msg[] = "UART Loopback Test\r\n";
//   HAL_UART_Transmit(&hcom_uart[COM1], test_msg, sizeof(test_msg)-1, HAL_MAX_DELAY);
  
//   // Send a test character on USART2
//   uint8_t test_char = 'L';
//   HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&BT_UART, &test_char, 1, 1000);
  
//   uint8_t result_msg[50];
//   sprintf((char*)result_msg, "TX Status: %d\r\n", tx_status);
//   HAL_UART_Transmit(&hcom_uart[COM1], result_msg, strlen((char*)result_msg), HAL_MAX_DELAY);
  
//   // Check if anything was received
//   HAL_Delay(100);
//   sprintf((char*)result_msg, "After TX - ISR: 0x%08lX\r\n", USART2->ISR);
//   HAL_UART_Transmit(&hcom_uart[COM1], result_msg, strlen((char*)result_msg), HAL_MAX_DELAY);
// }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  
  /* Note: System clock configuration is handled by CM7 core */
  /* CM4 core should not call SystemClock_Config() */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_MspInit();
  /* USER CODE BEGIN Init */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  // STM32H7 specific: Ensure USART2 is in the correct power domain
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
  
  // Wait for USART2 clock to stabilize
  HAL_Delay(1);
  Terminal_Console_Init();
  MX_TIM3_Init();
  //HAL_Delay(1);
  MX_TIM4_Init();

  MX_I2C4_Init();
  BlueTooth_Console_Init();

  // Initialize Motor Control System
  MotorControl_Init();
  MotorControl_ADC_Init();
  
  char motor_init_msg[] = "Motor Control and ADC Initialized!\r\n";
  Terminal_Display(motor_init_msg);

 HAL_StatusTypeDef statusI2C = HAL_OK;

  char timer_init_msg[100];
  uint8_t IMUAdd = 0x68<<1;
  uint16_t MemAddSize = 1;
  uint8_t pData = 0x04;
  uint8_t tData[2];
  uint16_t Size = 1;
  uint32_t Timeout = 100;
  int16_t temperature_raw = 0;
  float_t temperature = 0.0;

  HAL_I2C_Mem_Write(&hi2c1, IMUAdd, 0x6B, MemAddSize, &pData,  Size,  Timeout);
  
  
  while (1)
  {

  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    Terminal_Display("Hello from Error Handler\r\n");
    HAL_Delay(10000); // Delay to avoid flooding the UART
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
