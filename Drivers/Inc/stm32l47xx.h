/*
 * stm32l47xx.h
 *
 *  Created on: Mar 28, 2025
 *      Author: harshilpatel
 */

#ifndef INC_STM32L47XX_H_
#define INC_STM32L47XX_H_

#include <stdint.h>

#define __vo volatile


/*****************************************************************************************************************************************************************
 * Peripheral Memory Base Addresses
*****************************************************************************************************************************************************************/

/*
 * Base addresses for Flash and SRAM memory spaces
 */
#define FLASH_BASE_ADDR			(uint32_t)0x08000000
#define SRAM1_BASE_ADDR			(uint32_t)0x20000000	// 96 KB, 32-bit word length
#define SRAM2_BASE_ADDR			(uint32_t)0x10000000	// 32 KB, 32-bit word length (can also be aliased to 0x20040000, creating continuous address space with SRAM1)
#define SRAM 					SRAM1_BASE_ADDR
#define	ROM						(uint32_t)0x1FFF0000


/*
 * Base addresses for peripheral busses
 */
#define	PERIPH_BASE_ADDR		(uint32_t)0x40000000
#define	AHB1_BASE_ADDR			(uint32_t)0x40020000
#define AHB2_BASE_ADDR			(uint32_t)0x48000000
#define APB1_BASE_ADDR			PERIPH_BASE_ADDR
#define APB2_BASE_ADDR			(uint32_t)0x40010000


/*
 * Base addresses for peripherals on AHB1 bus
 */
#define DMA1_BASE_ADDR			AHB1_BASE_ADDR
#define DMA2_BASE_ADDR			(AHB1_BASE_ADDR + 0x0400)
#define RCC_BASE_ADDR			(AHB1_BASE_ADDR + 0x1000)


/*
 * Base addresses for GPIO ports on AHB2 bus
 */
#define GPIOA_BASE_ADDR			AHB2_BASE_ADDR
#define GPIOB_BASE_ADDR			(AHB2_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR			(AHB2_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR			(AHB2_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR			(AHB2_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR			(AHB2_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR			(AHB2_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR			(AHB2_BASE_ADDR + 0x1C00)


/*
 * Base addresses for peripherals on APB1 bus
 */
#define TIM2_BASE_ADDR			APB1_BASE_ADDR
#define TIM3_BASE_ADDR			(APB1_BASE_ADDR + 0x0400)
#define TIM4_BASE_ADDR			(APB1_BASE_ADDR + 0x0800)
#define TIM5_BASE_ADDR			(APB1_BASE_ADDR + 0x0C00)
#define TIM6_BASE_ADDR			(APB1_BASE_ADDR + 0x1000)
#define TIM7_BASE_ADDR			(APB1_BASE_ADDR + 0x1400)

#define SPI2_BASE_ADDR			(APB1_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR			(APB1_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR		(APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR		(APB1_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR			(APB1_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR			(APB1_BASE_ADDR + 0x5000)

#define I2C1_BASE_ADDR			(APB1_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR			(APB1_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR			(APB1_BASE_ADDR + 0x5C00)


/*
 * Base addresses for peripherals on APB2 bus
 */
#define SYSCFG_BASE_ADDR		APB2_BASE_ADDR
#define EXTI_BASE_ADDR			(APB2_BASE_ADDR + 0x0400)

#define SPI1_BASE_ADDR			(APB2_BASE_ADDR + 0x3000)

#define USART1_BASE_ADDR		(APB2_BASE_ADDR + 0x3800)

#define TIM1_BASE_ADDR			(APB2_BASE_ADDR + 0x2C00)
#define TIM8_BASE_ADDR			(APB2_BASE_ADDR + 0x3400)
#define TIM15_BASE_ADDR			(APB2_BASE_ADDR + 0x4000)
#define TIM16_BASE_ADDR			(APB2_BASE_ADDR + 0x4400)
#define TIM17_BASE_ADDR			(APB2_BASE_ADDR + 0x4800)


/*****************************************************************************************************************************************************************
 * Peripheral Register Definitions
*****************************************************************************************************************************************************************/

typedef struct
{
	__vo uint32_t MODER;		/* Port mode register												Address offset: 0x00 */
	__vo uint32_t OTYPER;		/* Output type register												Address offset: 0x04 */
	__vo uint32_t OSPEEDR;		/* Output speed register											Address offset: 0x08 */
	__vo uint32_t PUPDR;		/* Pull-up/Pull-down register										Address offset: 0x0C */
	__vo uint32_t IDR;			/* Input data register												Address offset: 0x10 */
	__vo uint32_t ODR;			/* Output data register												Address offset: 0x14 */
	__vo uint32_t BSRR;		/* Bit set/reset register											Address offset: 0x18 */
	__vo uint32_t LCKR;		/* Configuration lock register										Address offset: 0x1C */
	__vo uint32_t AFR[2];		/* Alternate function register; AFR[0] = ARFL, AFR[1] = AFRH		Address offset: 0x20 */
	__vo uint32_t BRR;			/* Bit reset register												Address offset: 0x28 */
	__vo uint32_t ASCR;		/* Analog switch control register									Address offset: 0x2C */
} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			/* Clock control register											Address offset: 0x00 */
	__vo uint32_t ICSCR;		/* Internal clock sources calibration control register				Address offset: 0x04 */
	__vo uint32_t CFGR;			/* Clock configuration register										Address offset: 0x08 */
	__vo uint32_t PLLCFGR;		/* PLL configuration register										Address offset: 0x0C */
	__vo uint32_t PLLSAI1CFGR;	/* PLLSAI1 configuration register									Address offset: 0x10 */
	__vo uint32_t PLLSAI2CFGR;	/* PLLSAI2 configuration register									Address offset: 0x14 */
	__vo uint32_t CIER;			/* Clock interrupt enable register									Address offset: 0x18 */
	__vo uint32_t CIFR;			/* Clock interrupt flag enable register								Address offset: 0x1C */
	__vo uint32_t CICR;			/* Clock interrupt clear register									Address offset: 0x20 */
	uint32_t 	  RESERVED0;	/* Reserved															Address offset: 0x24 */
	__vo uint32_t AHBIRSTR;		/* AHB1 peripheral reset register									Address offset: 0x28 */
	__vo uint32_t AHB2RSTR;		/* AHB2 peripheral reset register									Address offset: 0x2C */
	__vo uint32_t AHB3RSTR;		/* AHB3 peripheral reset register									Address offset: 0x30 */
	uint32_t 	  RESERVED1;	/* Reserved 														Address offset: 0x34 */
	__vo uint32_t APB1RSTR1;	/* APB1 peripheral reset register 1									Address offset: 0x38 */
	__vo uint32_t APB1RSTR2;	/* APB1 peripheral reset register 2									Address offset: 0x3C */
	__vo uint32_t APB2RSTR;		/* APB2 peripheral reset register									Address offset: 0x40 */
	uint32_t 	  RESERVED2;	/* Reserved 														Address offset: 0x44 */
	__vo uint32_t AHB1ENR;		/* AHB1 peripheral clock enable register							Address offset: 0x48 */
	__vo uint32_t AHB2ENR;		/* AHB2 peripheral clock enable register							Address offset: 0x4C */
	__vo uint32_t AHB3ENR;		/* AHB3 peripheral clock enable register							Address offset: 0x50 */
	uint32_t 	  RESERVED3;	/* Reserved 														Address offset: 0x54 */
	__vo uint32_t APB1ENR1;		/* APB1 peripheral clock enable register 1							Address offset: 0x58 */
	__vo uint32_t APB1ENR2;		/* APB1 peripheral clock enable register 2							Address offset: 0x5C */
	__vo uint32_t APB2ENR;		/* APB2 peripheral clock enable register							Address offset: 0x60 */
	uint32_t 	  RESERVED4;	/* Reserved 														Address offset: 0x64 */
	__vo uint32_t AHB1SMENR;	/* AHB1 peripheral clock enable in sleep and stop mode register		Address offset: 0x68 */
	__vo uint32_t AHB2SMENR;	/* AHB2 peripheral clock enable in sleep and stop mode register		Address offset: 0x6C */
	__vo uint32_t AHB3SMENR;	/* AHB3 peripheral clock enable in sleep and stop mode register		Address offset: 0x70 */
	uint32_t 	  RESERVED5;	/* Reserved 														Address offset: 0x74 */
	__vo uint32_t APB1SMENR1;	/* APB1 peripheral clock enable in sleep and stop mode register 1	Address offset: 0x78 */
	__vo uint32_t APB1SMENR2;	/* APB1 peripheral clock enable in sleep and stop mode register 2	Address offset: 0x7C */
	__vo uint32_t APB2SMENR;	/* APB2 peripheral clock enable in sleep and stop mode register		Address offset: 0x80 */
	uint32_t 	  RESERVED6;	/* Reserved 														Address offset: 0x84 */
	__vo uint32_t CCIPR;		/* Peripherals independent clock configuration register				Address offset: 0x88 */
	uint32_t 	  RESERVED7;	/* Reserved 														Address offset: 0x8C */
	__vo uint32_t BDCR;			/* Backup domain control register									Address offset: 0x90 */
	__vo uint32_t CSR;			/* Control/status register											Address offset: 0x94 */
	__vo uint32_t CRRCR;		/* Clock recovery RC register										Address offset: 0x98 */
	__vo uint32_t CCIPR2;		/* Peripherals independent clock configuration register 2			Address offset: 0x9C */
}RCC_RegDef_t;


/*
 * Peripheral definitions ( Peripheral base addresses typecast to xx_RegDef_t )
 */
#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

#define RCC						((GPIO_RegDef_t*)GPIOH_BASE_ADDR)



/*
 * Clock enable/disable MACROs for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 0 ))
#define GPIOA_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 0 ))

#define GPIOB_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 1 ))
#define GPIOB_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 1 ))

#define GPIOC_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 2 ))
#define GPIOC_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 2 ))

#define GPIOD_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 3 ))
#define GPIOD_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 3 ))

#define GPIOE_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 4 ))
#define GPIOE_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 4 ))

#define GPIOF_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 5 ))
#define GPIOF_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 5 ))

#define GPIOG_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 6 ))
#define GPIOG_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 6 ))

#define GPIOH_PCLK_EN()			(RCC->AHB2ENR |= ( 1 << 7 ))
#define GPIOH_PCLK_DIS()		(RCC->AHB2ENR &= ~( 1 << 7 ))

/*
 * Clock enable/disable MACROs for I2C peripherals
 */
#define I2C1_PCLCK_EN()			(RCC->APB1ENR1 |= ( 1 << 21 ))
#define I2C1_PCLCK_DIS()		(RCC->APB1ENR1 &= ~( 1 << 21 ))

#define I2C2_PCLCK_EN()			(RCC->APB1ENR1 |= ( 1 << 22 ))
#define I2C2_PCLCK_DIS()		(RCC->APB1ENR1 &= ~( 1 << 22 ))

#define I2C3_PCLCK_EN()			(RCC->APB1ENR1 |= ( 1 << 23 ))
#define I2C3_PCLCK_DIS()		(RCC->APB1ENR1 &= ~( 1 << 23 ))

/*
 * Clock enable/disable MACROs for SPI peripherals
 */
#define SPI1_PCLK_EN			(RCC->APB2ENR |= ( 1 << 12 ))
#define SPI1_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 12 ))

#define SPI2_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 14 ))
#define SPI2_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 14 ))

#define SPI3_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 15 ))
#define SPI3_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 15 ))

/*
 * Clock enable/disable MACROs for USART/UART peripherals
 */
#define USART1_PCLK_EN			(RCC->APB2ENR |= ( 1 << 14 ))
#define USART1_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 14 ))

#define USART2_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 17 ))
#define USART2_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 17 ))

#define USART3_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 18 ))
#define USART3_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 18 ))

#define UART4_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 19 ))
#define UART4_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 19 ))

#define UART5_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 20 ))
#define UART5_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 20 ))

/*
 * Clock enable/disable MACROs for Timer peripherals
 */
#define TIM1_PCLK_EN			(RCC->APB2ENR |= ( 1 << 11 ))
#define TIM1_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 11 ))

#define TIM2_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 0 ))
#define TIM2_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 0 ))

#define TIM3_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 1 ))
#define TIM3_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 1 ))

#define TIM4_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 2 ))
#define TIM4_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 2 ))

#define TIM5_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 3 ))
#define TIM5_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 3 ))

#define TIM6_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 4 ))
#define TIM6_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 4 ))

#define TIM7_PCLK_EN			(RCC->APB1ENR1 |= ( 1 << 5 ))
#define TIM7_PCLK_DIS			(RCC->APB1ENR1 &= ~( 1 << 5 ))

#define TIM8_PCLK_EN			(RCC->APB2ENR |= ( 1 << 13 ))
#define TIM8_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 13 ))

#define TIM15_PCLK_EN			(RCC->APB2ENR |= ( 1 << 16 ))
#define TIM15_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 16 ))

#define TIM16_PCLK_EN			(RCC->APB2ENR |= ( 1 << 17 ))
#define TIM16_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 17 ))

#define TIM17_PCLK_EN			(RCC->APB2ENR |= ( 1 << 18 ))
#define TIM17_PCLK_DIS			(RCC->APB2ENR &= ~( 1 << 18 ))

#endif /* INC_STM32L47XX_H_ */
