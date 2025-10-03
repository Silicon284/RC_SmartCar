#include "IMU_Control.h"

I2C_HandleTypeDef hi2c1;

void MX_I2C4_Init(void)
{
  __HAL_RCC_I2C1_CLK_ENABLE();
    
    /* USER CODE BEGIN I2C4_Init 0 */
	  /* USER CODE BEGIN I2C4_Init 0 */
	GPIO_InitTypeDef GPIO_InitStruct_I2C_SCL = {0};

  /* USER CODE BEGIN I2C1  */
  /*Configure GPIO pin : PB8(SCL)*/
	GPIO_InitStruct_I2C_SCL.Pin 		= GPIO_PIN_8;
	GPIO_InitStruct_I2C_SCL.Mode 		= GPIO_MODE_AF_OD;
	GPIO_InitStruct_I2C_SCL.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct_I2C_SCL.Alternate 	= GPIO_AF4_I2C1;
	GPIO_InitStruct_I2C_SCL.Speed 		= GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_I2C_SCL);
  /* USER CODE END TIM3_Init 0 */

  /* USER CODE BEGIN I2C4_Init 0 */
	GPIO_InitTypeDef GPIO_InitStruct_I2C_SDA = {0};

		  /* USER CODE BEGIN I2C1  */
		  /*Configure GPIO pin : PB9(SDA)*/
		GPIO_InitStruct_I2C_SDA.Pin 		= GPIO_PIN_9;
		GPIO_InitStruct_I2C_SDA.Mode 		= GPIO_MODE_AF_OD;
		GPIO_InitStruct_I2C_SDA.Pull 		= GPIO_NOPULL;
		GPIO_InitStruct_I2C_SDA.Alternate 	= GPIO_AF4_I2C1;
		GPIO_InitStruct_I2C_SDA.Speed 		= GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_I2C_SDA);
  /* USER CODE END I2C4 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00909FCE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}


