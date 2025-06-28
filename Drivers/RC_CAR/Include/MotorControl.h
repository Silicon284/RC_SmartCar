#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

# define MT_ENABLE 1

// Motor A Enable Pin
#define MT_A_EN_PIN                                GPIO_PIN_14
#define MT_A_GPIO_PORT                              GPIOB
#define MT_A_EN_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define MT_A_EN_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()
#define MT_A_EN_AF                                  GPIO_AF7_USART3

// Motor A Direction Control Pins
#define MT_A_IN1_PIN                               GPIO_PIN_15
#define MT_A_IN1_GPIO_PORT                         GPIOB
#define MT_A_IN1_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define MT_A_IN1_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()
#define MT_A_IN1_AF                                GPIO_AF0_SWJ  // GPIO function

#define MT_A_IN2_PIN                               GPIO_PIN_0
#define MT_A_IN2_GPIO_PORT                         GPIOC
#define MT_A_IN2_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define MT_A_IN2_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOC_CLK_DISABLE()
#define MT_A_IN2_AF                                GPIO_AF0_SWJ  // GPIO function

// Motor A Current Sense Pin
#define MT_A_SENSE_PIN                             GPIO_PIN_1
#define MT_A_SENSE_GPIO_PORT                       GPIOC
#define MT_A_SENSE_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define MT_A_SENSE_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()
#define MT_A_SENSE_AF                              GPIO_AF1_TIM2  // For ADC input

// Motor B Enable Pin
#define MT_B_EN_PIN                                GPIO_PIN_2
#define MT_B_EN_GPIO_PORT                          GPIOC
#define MT_B_EN_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()
#define MT_B_EN_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOC_CLK_DISABLE()
#define MT_B_EN_AF                                 GPIO_AF1_TIM2  // For PWM

// Motor B Direction Control Pins
#define MT_B_IN1_PIN                               GPIO_PIN_3
#define MT_B_IN1_GPIO_PORT                         GPIOC
#define MT_B_IN1_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define MT_B_IN1_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOC_CLK_DISABLE()
#define MT_B_IN1_AF                                GPIO_AF0_SWJ  // GPIO function

#define MT_B_IN2_PIN                               GPIO_PIN_4
#define MT_B_IN2_GPIO_PORT                         GPIOC
#define MT_B_IN2_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()
#define MT_B_IN2_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOC_CLK_DISABLE()
#define MT_B_IN2_AF                                GPIO_AF0_SWJ  // GPIO function

// Motor B Current Sense Pin
#define MT_B_SENSE_PIN                             GPIO_PIN_5
#define MT_B_SENSE_GPIO_PORT                       GPIOC
#define MT_B_SENSE_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOC_CLK_ENABLE()
#define MT_B_SENSE_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOC_CLK_DISABLE()
#define MT_B_SENSE_AF                              GPIO_AF1_TIM2  // For ADC input

