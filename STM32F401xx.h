/*
 * stm32f401xx.h
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_
#include <stdint.h>
#define __vo volatile
/*
 * Base address of FLASH and RAM memory
 */
#define FLASH_BASEADDR			0x0800 0000U
#define SRAM1_BASEADDR			0x2000 0000U
#define SRAM					SRAM1_BASEADDR
#define ROM_BASEADDR			0x1FFF 0000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE				0x4000 0000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U

#define AHB1PERIPH_BASE			0x40020000U

#define AHB2PERIPH_BASE			0x5000 0000U

/*
 * Base address of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE+0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE+0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE+0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE+0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE+0x1000)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE+0x1C00)
#define RCC_BASEADDR			(AHB1PERIPH_BASE+0x3800)

/*
 * Base address of peripherals which are hanging on APB1 Bus
 */
#define I2C1_BASE_ADDR			(APB1PERIPH_BASE+0x5400)
#define I2C2_BASE_ADDR			(APB1PERIPH_BASE+0x5800)
#define I2C3_BASE_ADDR			(APB1PERIPH_BASE+0x5400)
#define SPI2_BASE_ADDR			(APB1PERIPH_BASE+0x3800)
#define SPI3_BASE_ADDR			(APB1PERIPH_BASE+0x3C00)
#define USART2_BASE_ADDR		(APB1PERIPH_BASE+0x4400)

/*
 * Base address of peripheral which are hanging on APB2 Bus
 */
#define EXTI_BASE_ADDR			(APB2PERIPH_BASE+0x3C00)
#define SPI1_BASE_ADDR			(APB2PERIPH_BASE+0x3000)
#define SPI4_BASE_ADDR			(APB2PERIPH_BASE+0x3400)
#define SYSCFG_BASE_ADDR		(APB2PERIPH_BASE+0x3800)
#define USART1_BASE_ADDR		(APB2PERIPH_BASE+0x1000)
#define USART6_BASE_ADDR		(APB2PERIPH_BASE+0x1400)

/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;				// Address offset :0000
	__vo uint32_t OTYPER;            // Address offset:0x04
	__vo uint32_t OSPEEDR;           //Address offset:0x08
	__vo uint32_t PUPDR;             //Address offset:0x0C
	__vo uint32_t IDR;				 //Address offset:0x10
	__vo uint32_t ODR;				 //Address offset:0x14
	__vo uint32_t BSRR;				 //Address offset:0x18
	__vo uint32_t LCKR;				 //Address offset:0x1C
	__vo uint32_t AFR[2];			 //AFR[0]= GPIO FUNCTION LOW REGISTER AND AFR[1]= GPIO FUNCTION High REGISTER Address offset:0x20 -0x24

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;						 // Address offset:0x00
	__vo uint32_t PLLCFGR;					 // Address offset:0x04
	__vo uint32_t CFGR;						 // Address offset:0x08
	__vo uint32_t CIR;						 // Address offset:0x0C
	__vo uint32_t AHB1RSTR;					 // Address offset:0x10
	__vo uint32_t AHB2RSTR;					 // Address offset:0x14
	 uint32_t Reserved0[2];                  // Address offset:0x18-0x1C
	__vo uint32_t APB1RSTR;					 // Address offset:0x20
	__vo uint32_t APB2RSTR;					 // Address offset:0x24
	uint32_t Reserved1[2];                   // Address offset:0x28-0x2C
	__vo uint32_t AHB1ENR;					 // Address offset:0x30
	__vo uint32_t AHB2ENR;					 // Address offset:0x34
	uint32_t Reserved2[2];                   // Address offset:0x38-0x3C
	__vo uint32_t APB1ENR;					 // Address offset:0x40
	__vo uint32_t APB2ENR;					 // Address offset:0x44
	uint32_t Reserved3[2];                   // Address offset:0x48-0x4C
	__vo uint32_t AHB1LPENR;				 // Address offset:0x50
	__vo uint32_t AHB2LPENR;				 // Address offset:0x54
	uint32_t Reserved4[2];                   // Address offset:0x58-0x5C
	__vo uint32_t APB1LPENR;				 // Address offset:0x60
	__vo uint32_t APB2LPENR;				 // Address offset:0x64
	uint32_t Reserved5[2];                   // Address offset:0x68-0x6C
	__vo uint32_t BDCR;						 // Address offset:0x70
	__vo uint32_t CSR;						 // Address offset:0x74
	uint32_t Reserved6[2];                   // Address offset:0x78-0x7C
	__vo uint32_t SSCGR;				     // Address offset:0x80
	__vo uint32_t PLLI2SCFGR;				 // Address offset:0x84
	__vo uint32_t DCKCFGR;					 // Address offset:0x8C


}RCC_Reg_Def_t;
/*
 * peripheral definitions
 */
#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define RCC                         ((RCC_Reg_Def_t*)RCC_BASEADDR)
/*
 * Peripheral definitions
 */
/*
 * Clock Enable macro for GPIO peripheral
 */
//#define RCC_BASEADDR			(AHB1PERIPH_BASE+0x3800)
#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |=(1<<4))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |=(1<<5))
/*
 * Clock Enable macro for I2Cx peripheral
 */
#define I2C1_PCLK_EN()         (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()         (RCC->APB1ENR |=(1<<22))

/*
 * Clock Enable macro for SPIx peripheral
 */
#define SPI2_PCLK_EN()         (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()         (RCC->APB1ENR |=(1<<15))
#define SPI1_PCLK_EN()			(RCC->APB2ENR |=(1<<12))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |=(1<<13))

/*
 * Clock Enable macro for USARTx peripheral
 */
#define USART2_PCLK_EN()         (RCC->APB1ENR |=(1<<17))
#define USART1_PCLK_EN()         (RCC->APB2ENR |=(1<<4))
#define USART6_PCLK_EN()         (RCC->APB2ENR |=(1<<5))
// 84 LECTURE SERIES
/*
 * Clock Enable macro for SYSCFG peripheral
 */

# define SYSCFG_PCLK_EN()         (RCC->APB2ENR |=(1<<14))

/*
 * Clock Disable macro for GPIO peripheral
 */
#define GPIOA_PCLK_DSEN()         (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DSEN()         (RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DSEN()         (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DSEN()         (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DSEN()         (RCC->AHB1ENR &=~(1<<4))
#define GPIOH_PCLK_DSEN() 		  (RCC->AHB1ENR &=~(1<<4))

/*
 * Clock Disable macro for I2Cx peripheral
 */
#define I2C1_PCLK_DSEN()         (RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DSEN()         (RCC->APB1ENR &=~(1<<22))

/*
 * Clock Disable macro for SPIx peripheral
 */
#define SPI2_PCLK_DSEN()         (RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DSEN()         (RCC->APB1ENR &=~(1<<15))
#define SPI1_PCLK_DSEN()		 (RCC->APB2ENR &=~(1<<12))
#define SPI4_PCLK_DSEN()		 (RCC->APB2ENR &=~(1<<13))

/*
 * Clock Disable macro for USARTx peripheral
 */
#define USART2_PCLK_DSEN()         (RCC->APB1ENR &=~(1<<17))
#define USART1_PCLK_DSEN()         (RCC->APB2ENR &=~(1<<4))
#define USART6_PCLK_DSEN()         (RCC->APB2ENR &=~(1<<5))

/*
 * Clock Enable macro for SYSCFG peripheral
 */

# define SYSCFG_PCLK_EN()         (RCC->APB2ENR |=(1<<14))

/*
 * Macro to reset GPIO GPIOx peripherals
 */


#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

#define EN    								1
#define DSEN  								0
#define SET   								EN
#define RESET           					DSEN
#define GPIO_PIN_SET    					SET
# define GPIO_PIN_RESET 					RESET
//GPIO_RegDef_t *pGPIOA =(GPIO_RegDef_t*)0x40020000;

// 92  LECTUREs completed
//#include "STM32F401_GPIO.h"
#include "STM32F401_gpio.h"
#endif /* INC_STM32F401XX_H_ */
