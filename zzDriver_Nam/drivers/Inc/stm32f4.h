#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#define __vo volatile
#include <stdint.h>

//NVIC ISERx core
#define NVIC_ISER0 ((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t *)0xE000E10c)

//NVIC ICERx core
#define NVIC_ICER0 ((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1 ((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3 ((__vo uint32_t *)0xE000E18c)

#define NVIC_PR_BASE_ADDR ((__vo uint32_t *)0xE000E400)

//buoc 1
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C0000U
#define ROM_BASEADDR	0x1FFF0000U
#define SRAM SRAM1_BASEADDR

// buoc 2 config periph
#define PERIPH_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

//buoc 3 CONFIG AHB1

#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

// BUOC 4 CONFIG APB1
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000)

//BUOC 5 CONFIG APB2
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)

//BUOC 6 gpio
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	//uint32_t AFRH;
}GPIO_RegDef_t;
// buoc 7
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)

// buoc 8 RCC
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;

	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;

	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
	//uint32_t AFRH;
}RCC_RegDef_t;
#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
//BUOC 9 EXTERNAL
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXIT_RegDef_t;
#define EXTI ((EXIT_RegDef_t *)EXTI_BASEADDR)

// buoc 10 spi

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;
#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)

// BUOC 11 SYSTEM CONFIG

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVERD1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVERD2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

// BUOC 12 I2C

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2CRegDef_t;
#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

// BUOC 13 UART

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USARTRegDef_t;
#define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define UART4 ((USART_RegDef_t *)UART1_BASEADDR)
#define UART5 ((USART_RegDef_t *)UART2_BASEADDR)
#define USART6 ((USART_RegDef_t *)USART6_BASEADDR)

// buoc 14 ENABLEclock cho gpio
#define GPIOA_PCLK_EN()(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()(RCC->AHB1ENR |= (1<<8))

#define I2C1_PCLK_EN()(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()(RCC->APB1ENR |= (1<<23))

#define SPI1_PCLK_EN()(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()(RCC->APB2ENR |= (1<<13))

#define USART1_PCLK_EN()(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()(RCC->APB2ENR |= (1<<5))

#define SYSCFG_PCLK_EN()(RCC->APB2ENR |= (1<<14))

// DISABLECLOCK

#define GPIOA_PCLK_DI()(RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()(RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()(RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()(RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()(RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()(RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()(RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()(RCC->AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI()(RCC->AHB1ENR &=~(1<<8))

#define I2C1_PCLK_DI()(RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()(RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()(RCC->APB1ENR &=~(1<<23))

#define SPI1_PCLK_DI()(RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()(RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()(RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DI()(RCC->APB2ENR &=~(1<<13))

#define USART1_PCLK_DI()(RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DI()(RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI()(RCC->APB1ENR &=~(1<<18))
#define UART4_PCLK_DI()(RCC->APB1ENR &=~(1<<19))
#define UART5_PCLK_DI()(RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DI()(RCC->APB2ENR &=~(1<<5))

#define SYSCFG_PCLK_DI()(RCC->APB2ENR &=~(1<<14))

//
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET RESET
#define FLAG_SET SET

#define GPIOA_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<0));	(RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<1));	(RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<2));	(RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<3));	(RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<4));	(RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOF_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<5));	(RCC->AHB1RSTR &= ~(1<<5));} while(0)
#define GPIOG_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<6));	(RCC->AHB1RSTR &= ~(1<<6));} while(0)
#define GPIOH_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<7));	(RCC->AHB1RSTR &= ~(1<<7));} while(0)
#define GPIOI_REG_RESET() 	do{(RCC->AHB1RSTR |= (1<<8));	(RCC->AHB1RSTR &= ~(1<<8));} while(0)


#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)? 0:(x == GPIOB)? 1:(x == GPIOC)? 2:(x == GPIOD)? 3:(x == GPIOE)? 4:(x == GPIOF)? 5: (x == GPIOG)? 6: (x == GPIOH)? 7: (x == GPIOI)? 8: 0)

#define IRQ_NO_EXTI0 	6
#define IRQ_NO_EXTI1	7
#define IRQ_NO_EXTI2 	8
#define IRQ_NO_EXTI3	9
#define IRQ_NO_EXTI4	10
#define IRQ_NO_EXTI9_5	23
#define IRQ_NO_EXTI15_10	40



#endif  /*INC_STM32F407_H_*/
