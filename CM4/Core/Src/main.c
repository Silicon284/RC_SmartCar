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
#include "stm32h7xx_nucleo.h"
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

COM_InitTypeDef BspCOMInit;
UART_HandleTypeDef BT_UART;
UART_InitTypeDef BT_COMInit;
GPIO_InitTypeDef BT_gpio_init_structure;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

/*USART2-BLUETOOTH*/
 
#define BT_COM1_UART                     USART2
#define BT_COM1_CLK_ENABLE()             __HAL_RCC_USART2_CLK_ENABLE()
#define BT_COM1_CLK_DISABLE()            __HAL_RCC_USART2_CLK_DISABLE()

#define BT_COM1_TX_PIN                   GPIO_PIN_5
#define BT_COM1_TX_GPIO_PORT             GPIOD
#define BT_COM1_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define BT_COM1_TX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOD_CLK_DISABLE()
#define BT_COM1_TX_AF                    GPIO_AF7_USART2

#define BT_COM1_RX_PIN                   GPIO_PIN_6
#define BT_COM1_RX_GPIO_PORT             GPIOD
#define BT_COM1_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOD_CLK_ENABLE()
#define BT_COM1_RX_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOD_CLK_DISABLE()
#define BT_COM1_RX_AF                    GPIO_AF7_USART2
#define BT_COM_POLL_TIMEOUT             1000
   /* Enable GPIO clock */
  BT_COM1_TX_GPIO_CLK_ENABLE();
  BT_COM1_RX_GPIO_CLK_ENABLE();

  /* Enable USART clock */
  BT_COM1_CLK_ENABLE();

  /* Configure USART Tx as alternate function */
  BT_gpio_init_structure.Pin = BT_COM1_TX_PIN;
  BT_gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  BT_gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
  BT_gpio_init_structure.Pull = GPIO_NOPULL;
  BT_gpio_init_structure.Alternate = BT_COM1_TX_AF;
  HAL_GPIO_Init(BT_COM1_TX_GPIO_PORT, &BT_gpio_init_structure);

  /* Configure USART Rx as alternate function */
  BT_gpio_init_structure.Pin = BT_COM1_RX_PIN;
  BT_gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  BT_gpio_init_structure.Alternate = BT_COM1_RX_AF;
  HAL_GPIO_Init(BT_COM1_RX_GPIO_PORT, &BT_gpio_init_structure);
   /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */

  BT_UART.Instance       = USART2;
  BT_UART.Init.BaudRate   = 38400;
  BT_UART.Init.WordLength = COM_WORDLENGTH_8B;
  BT_UART.Init.StopBits   = COM_STOPBITS_1;
  BT_UART.Init.Parity     = COM_PARITY_NONE;
  BT_UART.Init.HwFlowCtl  = COM_HWCONTROL_NONE;
   BT_UART.Init.Mode         = UART_MODE_TX_RX;
   BT_UART.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&BT_UART) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }




uint8_t text[] = "Hello from STM32 On-Board USART MODULE!\r\n";
uint8_t text1[] = "AT";
uint8_t text2[10];
  /* Infinite loop */
  HAL_UART_Transmit(&hcom_uart[COM1], text, sizeof(text) - 1, HAL_MAX_DELAY);
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle LED on GPIOE pin 1    
    
    // while(HAL_UART_Receive(&hcom_uart[COM1],  text2, 1, HAL_MAX_DELAY) != HAL_OK)
    // {
    //   // Wait for data to be received
    // };
    // HAL_UART_Transmit(&hcom_uart[COM1], text2, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&BT_UART, text1, sizeof(text1)-1, HAL_MAX_DELAY);
    while(HAL_UART_Receive(&BT_UART,  text2, 2, HAL_MAX_DELAY) != HAL_OK)
    {
      // Wait for data to be received
    };
    // HAL_UART_Transmit(&BT_UART, text, 1, HAL_MAX_DELAY);

    HAL_Delay(1000); // Delay for 1000 milliseconds
    HAL_UART_Transmit(&hcom_uart[COM1], text2, 2, HAL_MAX_DELAY);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)"Hello from Error Handler\r\n", 22, HAL_MAX_DELAY);
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
