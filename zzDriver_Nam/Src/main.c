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
#include <stm32f4.h>
#include "stm32_gpio.h"

void delay(void)
{
	for(uint32_t i=0;i<500000/2;i++);
}

int a =0;

int main(void)
{

    /* Loop forever */
	GPIO_Handle_t GPIO_LED;
	GPIO_LED.pGPIOx = GPIOB;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIO_LED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC,GPIO_PIN_NO_11);
		//GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_11);
		//delay();
		delay();
	}
	return 0;
}


