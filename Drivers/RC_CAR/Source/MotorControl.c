#include "MotorControl.h"

GPIO_InitTypeDef Motor_gpio_structure;

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
    Motor_gpio_structure.Alternate = MT_A_SENSE_AF;
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
    Motor_gpio_structure.Alternate = MT_B_SENSE_AF;
    HAL_GPIO_Init(MT_B_SENSE_GPIO_PORT, &Motor_gpio_structure);
    
    // Set initial states - Motors disabled
    HAL_GPIO_WritePin(MT_A_IN1_GPIO_PORT, MT_A_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT_A_IN2_GPIO_PORT, MT_A_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT_B_IN1_GPIO_PORT, MT_B_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MT_B_IN2_GPIO_PORT, MT_B_IN2_PIN, GPIO_PIN_RESET);
}