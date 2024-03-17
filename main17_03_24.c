/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

//#include"stm32f401xx.h"
//#include"../Driver/Inc/STM32F401_gpio.h"
#include "../Driver/Inc/stm32f401xx.h"

void delay(void)
{
	uint32_t  i;
	for (i=0; i<640000;i++);
	//for (i=0; i<640000;i++);
	//for (i=0; i<640000;i++);
	//for (i=0; i<640000;i++);
	//for (i=0; i<640000;i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx =GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed =GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPudControl=GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,EN);

    GPIO_Init(&GpioLed);

    /* Loop forever */
	//for(;;);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();

	}
	return 0;
}
