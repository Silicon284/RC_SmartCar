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
#include <stdio.h>
#include <string.h>
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
uint8_t received_char[2];
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
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

  // STM32H7 specific: Ensure USART2 is in the correct power domain
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
  
  // Wait for USART2 clock to stabilize
  HAL_Delay(1);



  /*ENABLE INTERRUPT*/
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);  // Changed priority to 5 (lower priority than system)
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  BT_gpio_init_structure.Pull = GPIO_PULLUP;
  BT_gpio_init_structure.Alternate = BT_COM1_RX_AF;
  HAL_GPIO_Init(BT_COM1_RX_GPIO_PORT, &BT_gpio_init_structure);
   /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */

  BT_UART.Instance       = USART2;
  BT_UART.Init.BaudRate   = 115200;
  BT_UART.Init.WordLength = UART_WORDLENGTH_8B;
  BT_UART.Init.StopBits   = UART_STOPBITS_1;
  BT_UART.Init.Parity     = UART_PARITY_NONE;
  BT_UART.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
   BT_UART.Init.Mode         = UART_MODE_TX_RX;
   BT_UART.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&BT_UART) != HAL_OK)
  {
    Error_Handler();
  }

  // Verify UART is ready
  if (BT_UART.gState != HAL_UART_STATE_READY)
  {
    Error_Handler();
  }

  // Debug: Check UART registers
  uint8_t reg_debug[100];
  sprintf((char*)reg_debug, "USART2 CR1=0x%08lX, CR2=0x%08lX, RQR=0x%08lX\r\n", 
          USART2->CR1, USART2->CR2, USART2->RQR);
  HAL_UART_Transmit(&hcom_uart[COM1], reg_debug, strlen((char*)reg_debug), HAL_MAX_DELAY);
  
  sprintf((char*)reg_debug, "USART2 ISR=0x%08lX, RDR=0x%08lX\r\n", 
          USART2->ISR, USART2->RDR);
  HAL_UART_Transmit(&hcom_uart[COM1], reg_debug, strlen((char*)reg_debug), HAL_MAX_DELAY);

  // Debug: Check GPIO configuration
  sprintf((char*)reg_debug, "GPIOD MODER=0x%08lX, AFRL=0x%08lX, AFRH=0x%08lX\r\n", 
          GPIOD->MODER, GPIOD->AFR[0], GPIOD->AFR[1]);
  HAL_UART_Transmit(&hcom_uart[COM1], reg_debug, strlen((char*)reg_debug), HAL_MAX_DELAY);
  
  sprintf((char*)reg_debug, "GPIOD PUPDR=0x%08lX, IDR=0x%08lX\r\n", 
          GPIOD->PUPDR, GPIOD->IDR);
  HAL_UART_Transmit(&hcom_uart[COM1], reg_debug, strlen((char*)reg_debug), HAL_MAX_DELAY);

uint8_t text[] = "Hello from STM32 On-Board USART MODULE!\r\n";
//uint8_t text1[] = "CHECK_BT_TX\r\n";
//uint8_t text2[10];
  /* Infinite loop */
  HAL_UART_Transmit(&hcom_uart[COM1], text, sizeof(text) - 1, HAL_MAX_DELAY);
  // HAL_UART_RegisterCallback();
  
  // Check if UART receive interrupt setup is successful
  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&BT_UART, received_char, 2);
  if (status != HAL_OK)
  {
    uint8_t error_msg[] = "UART Receive IT Failed!\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], error_msg, sizeof(error_msg) - 1, HAL_MAX_DELAY);
    Error_Handler();
  }
  else
  {
    uint8_t success_msg[] = "UART Receive IT Started Successfully!\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], success_msg, sizeof(success_msg) - 1, HAL_MAX_DELAY);
  }

  // Test UART hardware configuration
  // test_uart_loopback();
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle LED on GPIOE pin 1    
    
    // Debug: Check UART state and hardware registers every 5 seconds
    // static uint32_t debug_counter = 0;
    // debug_counter++;
    // if (debug_counter >= 5)
    // {
    //   debug_counter = 0;
    //   uint8_t debug_state[100];
    //   sprintf((char*)debug_state, "UART State: gState=%lu, RxState=%lu\r\n", 
    //           (unsigned long)BT_UART.gState, (unsigned long)BT_UART.RxState);
    //   HAL_UART_Transmit(&hcom_uart[COM1], debug_state, strlen((char*)debug_state), HAL_MAX_DELAY);
      
      // Check UART hardware registers for incoming data
      // sprintf((char*)debug_state, "USART2 ISR=0x%08lX, RXNE=%d, ORE=%d\r\n", 
      //         USART2->ISR, 
      //         (USART2->ISR & USART_ISR_RXNE_RXFNE) ? 1 : 0,
      //         (USART2->ISR & USART_ISR_ORE) ? 1 : 0);
      // HAL_UART_Transmit(&hcom_uart[COM1], debug_state, strlen((char*)debug_state), HAL_MAX_DELAY);
      
      // // Check if receive interrupt is enabled
      // sprintf((char*)debug_state, "USART2 CR1=0x%08lX, RXNEIE=%d, UE=%d\r\n", 
      //         USART2->CR1,
      //         (USART2->CR1 & USART_CR1_RXNEIE_RXFNEIE) ? 1 : 0,
      //         (USART2->CR1 & USART_CR1_UE) ? 1 : 0);
      // HAL_UART_Transmit(&hcom_uart[COM1], debug_state, strlen((char*)debug_state), HAL_MAX_DELAY);
      
      // Test: Try to send a character on USART2 to test TX
      // uint8_t test_char = 'X';
      // HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&BT_UART, &test_char, 1, 1000);
      // sprintf((char*)debug_state, "USART2 TX Test: Status=%d\r\n", tx_status);
      // HAL_UART_Transmit(&hcom_uart[COM1], debug_state, strlen((char*)debug_state), HAL_MAX_DELAY);
      
      // Remove the manual callback test since we know it works
    // }
    
    // Removed redundant HAL_UART_Receive_IT call - it's handled in the callback
    HAL_Delay(1000); // Delay for 1000 milliseconds

    ///HAL_UART_Transmit(&hcom_uart[COM1], text2, sizeof(text2)-1, HAL_MAX_DELAY);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) // Check if the interrupt is from USART2
  {
    // Toggle LED to show interrupt is working
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Green LED
    
    uint8_t debug_msg[] = "ISR: Received '";
    HAL_UART_Transmit(&hcom_uart[COM1], debug_msg, sizeof(debug_msg)-1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&hcom_uart[COM1], received_char, 2, HAL_MAX_DELAY); // Echo received character
    uint8_t end_msg[] = "'\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], end_msg, sizeof(end_msg)-1, HAL_MAX_DELAY);
    
    // Continue receiving in interrupt mode
    HAL_UART_Receive_IT(&BT_UART, received_char, 2);
  }
}

// Add error callback for debugging
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uint8_t error_msg[] = "UART Error Occurred during ISR!\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], error_msg, sizeof(error_msg)-1, HAL_MAX_DELAY);    // Debug: Check UART registers


    // Clear error and restart reception
    __HAL_UART_CLEAR_OREFLAG(&BT_UART);
    __HAL_UART_CLEAR_NEFLAG(&BT_UART);
    __HAL_UART_CLEAR_FEFLAG(&BT_UART);
    __HAL_UART_CLEAR_PEFLAG(&BT_UART);
    

    // Restart reception
    HAL_UART_Receive_IT(&BT_UART, received_char, 2);
  }
}
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
