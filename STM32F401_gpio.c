/*
 * STM32F401_gpio.c
 *
 *  Created on: Oct 23, 2023
 *      Author: Lenovo
 */

#include "STM32F401_gpio.h"
/*
 * Peripheral Clock setup
 */

/************************************************
 * @fn                   GPIO_PeriClockControl
 * @brief				-	This function enables or disables peripheral clock for the given GPIO PORT
 * @param [in]			-	base address of gpio peripheral
 * @param [in]			-	ENABLE or DISABLE macros
 * @return 				-   None
 * @Note 				-	None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == EN)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx ==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx ==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else
		{
			if(pGPIOx == GPIOA)
					{
						GPIOA_PCLK_DSEN();
					}
					else if (pGPIOx ==GPIOB)
					{
						GPIOB_PCLK_DSEN();
					}
					else if(pGPIOx == GPIOC)
					{
						GPIOC_PCLK_DSEN();
					}
					else if (pGPIOx ==GPIOD)
					{
						GPIOD_PCLK_DSEN();
					}
					else if(pGPIOx == GPIOE)
					{
						GPIOE_PCLK_DSEN();
					}
					else if (pGPIOx == GPIOH)
					{
						GPIOH_PCLK_DSEN();
					}
		}
	}
	else
	{
	}

}
/*
 * Init and D_Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
    // 1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	    //pGPIOHandle->pGPIOx->MODER &=~(0x3 << GPIO_PinConfig.GPIO_PinNumber); //clearing pin
		pGPIOHandle->pGPIOx->MODER =temp;

	}
	else
	{
                     //do it later inteerupt
	}
	temp=0;
	// 2. configure the speed
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing pin
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp=0;
	//3. configure the pupd settings
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPudControl <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing pin
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp=0;
	//4. configure the optype
	temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing pin
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp=0;
	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALT_FN)
	{
		//CONFIGURE THE ALTERNATE
       uint8_t temp1, temp2;
       temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
       temp2 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
       pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4*temp2));
       pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}
void GPIO_DeInit(GPIO_Handle_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx ==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx ==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}
/*
 * Data Read and Write
 * @return 0 or 1
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	    uint16_t value;
		value = (uint16_t)pGPIOx->IDR;
	    return value;

}

 void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit bit corresponding to that pin
		pGPIOx->ODR |= (1 << PinNumber);

	}
	else
	{
		//write 0
		pGPIOx->ODR &= ~(1<< PinNumber);
	}

}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR =Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}
/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
