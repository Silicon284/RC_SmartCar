#include "Ultrasonic_Sensor.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim31;
TIM_HandleTypeDef htim32;

void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* USER CODE BEGIN TIM3_Init 0 */
	GPIO_InitTypeDef GPIO_InitStruct_TM3 = {0};

	  /* USER CODE BEGIN TIM3  */
	  /*Configure GPIO pin : PB0(LED1) or PC8 */
	GPIO_InitStruct_TM3.Pin 		= GPIO_PIN_0;
	GPIO_InitStruct_TM3.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct_TM3.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct_TM3.Alternate 	= GPIO_AF2_TIM3;
	GPIO_InitStruct_TM3.Speed 		= GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_TM3);
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance 				= TIM3;
  htim3.Channel 				= HAL_TIM_ACTIVE_CHANNEL_3;
  htim3.Init.Prescaler 			= 2000;
  htim3.Init.CounterMode 		= TIM_COUNTERMODE_UP;
  htim3.Init.Period 			= 50000;
  htim3.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode 		= TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger 	= TIM_TS_ITR0;
  sSlaveConfig.TriggerPolarity 	= TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter 	= 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode 	= TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode 			= TIM_OCMODE_PWM1;
  sConfigOC.Pulse 			= 16000;
  sConfigOC.OCPolarity 		= TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity 	= 0;
  sConfigOC.OCNIdleState 	= 0;
  sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_RESET;
  sConfigOC.OCFastMode 		= TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

	/* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	/* USER CODE END TIM3_Init 2 */
}