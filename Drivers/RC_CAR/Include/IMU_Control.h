#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include <stdio.h>
#include "Bluetooth_Console.h"
#include "main.h"
#include "stm32h7xx_hal_i2c.h"

void MX_I2C4_Init(void);

extern I2C_HandleTypeDef hi2c1;