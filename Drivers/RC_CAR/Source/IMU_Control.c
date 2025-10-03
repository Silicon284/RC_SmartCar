#include "IMU_Control.h"
#include <math.h>

void IMU_Euler_Angles(void);

I2C_HandleTypeDef hi2c1;

static int16_t ax, ay, az, gx, gy, gz, temp;


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
  /* USER CODE BEGIN I2C4_Init 0 */
	GPIO_InitTypeDef GPIO_InitStruct_IMU_GPIO = {0};
  /* USER CODE END I2C1_Init 2 */
  		  /* USER CODE BEGIN I2C1  */
		  /*Configure GPIO pin : PB0(SDA)*/
		GPIO_InitStruct_IMU_GPIO.Pin 		= GPIO_PIN_0;
		GPIO_InitStruct_IMU_GPIO.Mode 	= GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct_IMU_GPIO.Pull 	= GPIO_NOPULL;
		GPIO_InitStruct_IMU_GPIO.Speed 	= GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_IMU_GPIO);

      /*ENABLE INTERRUPT*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);  // Changed priority to 5 (lower priority than system)
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    uint8_t IMUAdd = 0x68<<1;
    uint16_t MemAddSize = 1;
    uint8_t pData = 0x01;
    
    uint16_t Size = 1;
    uint32_t Timeout = 100;

    HAL_I2C_Mem_Write(&hi2c1, IMUAdd, 0x38, MemAddSize, &pData,  Size,  Timeout);
    pData = 250;
    HAL_I2C_Mem_Write(&hi2c1, IMUAdd, 0x19, MemAddSize, &pData,  Size,  Timeout);

}

void IMU_Raw_Read(void) {

    uint8_t IMUAdd = 0x68<<1;
    uint16_t MemAddSize = 1;
    uint8_t pData = 0x24;
    
    uint16_t Size = 1;
    uint32_t Timeout = 100;

    //HAL_I2C_Mem_Write(&hi2c1, IMUAdd, 0x6B, MemAddSize, &pData,  Size,  Timeout);
    
    uint8_t IMUData[14];
    uint16_t IMU_bytes = 14;

    HAL_I2C_Mem_Read(&hi2c1, IMUAdd, 0x3B, MemAddSize, &IMUData[0],  IMU_bytes,  Timeout);
    ax = IMUData[0] << 8 | IMUData[1];
    ay = IMUData[2] << 8 | IMUData[3];
    az = IMUData[4] << 8 | IMUData[5];

    temp = IMUData[6] << 8 | IMUData[7];

    gx = IMUData[8] << 8 | IMUData[9];
    gy = IMUData[10] << 8 | IMUData[11];
    gz = IMUData[12] << 8 | IMUData[13];

    pData = 0;
    //HAL_I2C_Mem_Write(&hi2c1, IMUAdd, 0x6B, MemAddSize, &pData,  Size,  Timeout);

    IMU_Euler_Angles();

}


void IMU_Euler_Angles(void) {

  char IMU_msgs[100];
  float_t IMU_Pitch = 0.0, IMU_Roll = 0.0;
  // Calculate pitch (rotation around Y-axis) 
  IMU_Pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
  
  // Calculate roll (rotation around X-axis)
  IMU_Roll = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;

  sprintf (IMU_msgs, "IMU Ax: %d IMU_Ay: %d\n\r", (int16_t)IMU_Pitch, (int16_t)IMU_Roll);
  Terminal_Display(IMU_msgs);
  
}
/*
void calculate_angles_complementary(float ax, float ay, float az, 
                                   float gx, float gy, float gz,
                                   float *pitch, float *roll, float *yaw)
{
    // Convert gyroscope from degrees/sec to radians/sec
    float gx_rad = gx * M_PI / 180.0f;
    float gy_rad = gy * M_PI / 180.0f;
    float gz_rad = gz * M_PI / 180.0f;
    
    // Calculate angles from accelerometer (in degrees)
    float pitch_accel = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    float roll_accel = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
    
    // Integrate gyroscope data (in degrees)
    float pitch_gyro = pitch_filtered + gy * SAMPLE_TIME;
    float roll_gyro = roll_filtered + gx * SAMPLE_TIME;
    
    // Apply complementary filter
    pitch_filtered = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;
    roll_filtered = ALPHA * roll_gyro + (1.0f - ALPHA) * roll_accel;
    
    // Yaw integration (gyroscope only, as accelerometer can't measure yaw)
    yaw_integrated += gz * SAMPLE_TIME;
    
    // Return filtered values
    *pitch = pitch_filtered;
    *roll = roll_filtered;
    *yaw = yaw_integrated;
}
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  
  char IMU_msgs[100];
  sprintf (IMU_msgs, "IMU in Callback");
  //Terminal_Display(IMU_msgs);
  
  IMU_Raw_Read();
}