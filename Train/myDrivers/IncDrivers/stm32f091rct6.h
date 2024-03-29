/*
 * stm32f091rct6.h
 *
 *  Created on: May 18, 2022
 *      Author: danim
 */

#ifndef INCDRIVERS_STM32F091RCT6_H_
#define INCDRIVERS_STM32F091RCT6_H_

#include <stdint.h>
#include <stddef.h>
/*****************************************ADDRESSES*****************************************/

/*BASE ADDRESSES OF MEMORIES: FLASH AND SRAM */
#define FLASH_BASE				0x08000000U
#define SRAM_BASE				0x20000000U
#define FLASH_SYSMEM_BASE		0x1FFFD800U

/*BASE ADDRESSES OF BUSES */
#define PERIPH_BUS_BASE			0x40000000U
#define APB_BASE				PERIPH_BUS_BASE
#define AHB1_BASE				0x40020000U
#define AHB2_BASE				0x48000000U

/*BASE ADDRESSES PERIPHERALS APB BUS*/
#define TIM2_BASE				APB_BASE
#define TIM3_BASE				(APB_BASE+0x0400)
#define TIM6_BASE				(APB_BASE+0x1000)
#define TIM7_BASE				(APB_BASE+0x1400)
#define TIM14_BASE				(APB_BASE+0x2000)
#define RTC_BASE				(APB_BASE+0x2800)
#define WWDG_BASE				(APB_BASE+0x2C00)
#define IWDG_BASE				(APB_BASE+0x2C00)
#define SPI2_BASE				(APB_BASE+0x3800)
#define USART2_BASE				(APB_BASE+0x4400)
#define USART3_BASE				(APB_BASE+0x4800)
#define USART4_BASE				(APB_BASE+0x4C00)
#define USART5_BASE				(APB_BASE+0x5000)
#define I2C1_BASE				(APB_BASE+0x5400)
#define I2C2_BASE				(APB_BASE+0x5800)
#define USB_BASE				(APB_BASE+0x5C00)
#define USB_CAN_SRAM_BASE		(APB_BASE+0x6000)
#define CAN_BASE				(APB_BASE+0x6400)
#define CRS_BASE				(APB_BASE+0x6C00)
#define PWR_BASE				(APB_BASE+0x7000)
#define DAC_BASE				(APB_BASE+0x7400)
#define CEC_BASE				(APB_BASE+0x7800)
#define SYSCFG_COMP_BASE		(APB_BASE+0x10000)
#define EXTI_BASE				(APB_BASE+0x10400)
#define USART6_BASE				(APB_BASE+0x11400)
#define USART7_BASE				(APB_BASE+0x11800)
#define USART8_BASE				(APB_BASE+0x11C00)
#define ADC_BASE				(APB_BASE+0x12400)
#define TIM1_BASE				(APB_BASE+0x12C00)
#define SPI1_I2S1_BASE			(APB_BASE+0x13000)
#define USART1_BASE				(APB_BASE+0x13800)
#define TIM15_BASE				(APB_BASE+0x14000)
#define TIM16_BASE				(APB_BASE+0x14400)
#define TIM17_BASE				(APB_BASE+0x14800)
#define DBGMCU_BASE				(APB_BASE+0x15800)

/*BASE ADDRESSES PERIPHERALS AHB1 BUS*/
#define DMA_BASE				AHB1_BASE
#define DMA2_BASE				(AHB1_BASE+0x0400)
#define RCC_BASE				(AHB1_BASE+0x1000)
#define FLASH_INT_BASE			(AHB1_BASE+0x2000)
#define CRC_BASE				(AHB1_BASE+0x3000)
#define TSC_BASE				(AHB1_BASE+0x4000)

/*BASE ADDRESSES PERIPHERALS AHB2 BUS*/
#define GPIOA_BASE				AHB2_BASE
#define GPIOB_BASE				(AHB2_BASE+0x0400)
#define GPIOC_BASE				(AHB2_BASE+0x0800)
#define GPIOD_BASE				(AHB2_BASE+0x0C00)
#define GPIOE_BASE				(AHB2_BASE+0x1000)
#define GPIOF_BASE				(AHB2_BASE+0x1400)

/*********************NVIC RELATED (Nested Vectors Interrupt Controller)***********************************/
/*I think it's better not to create a struct(non consecutive addresses). Check the M0 user guide for NVIC info */
/*ARM Cortex M0 ISER*/
#define NVIC_ISER         ( (volatile uint32_t*)0xE000E100 )
/*ARM Cortex M0 ICER*/
#define NVIC_ICER         ( (volatile uint32_t*)0xE000E180 )
/*ARM Cortex M0 IPR base*/
#define NVIC_IPR_BASE 	( (volatile uint32_t*)0xE000E400 )

/*IRQ numbers Nucleo STM32F091RC*/
#define IRQ_EXTI0_1		5
#define IRQ_EXTI2_3 		6
#define IRQ_EXTI4_15 		7
#define TIM1_CC		14
#define TIM2		15
#define TIM3		16
#define TIM6_DAC	17
#define TIM7		18
#define TIM14		19
#define TIM15		20
#define TIM16		21
#define TIM17		22
#define IRQ_I2C1    23
#define IRQ_I2C2     24
#define IRQ_SPI1			25
#define IRQ_SPI2         26
#define IRQ_USART1	    27
#define IRQ_USART2	    28
#define IRQ_USART3_TO_8	    29

/******************************************GENERIC MACROS**************************************************/
#define DISABLE					0
#define ENABLE					1
#define FLAG_NOT_SET			0
#define FLAG_SET				1

/******************************************STRUCTURES OF REGISTERS*****************************************/
/************************CLOCK*******************/
/*struct*/
typedef struct{
	volatile uint32_t CR;            /* Clock control register,     				Address offset: 0x00 */
	volatile uint32_t CFGR;       /* Clock config register 1,     Address offset: 0x04 (due to 4 bytes uint32_t) */
	volatile uint32_t CIR;          /* Clock interrupt register,     				Address offset: 0x08 */
	volatile uint32_t APB2RSTR;           /* Reset APB2,     						Address offset: 0x0C */
	volatile uint32_t APB1RSTR;      /* Reset APB1,     							Address offset: 0x10 */
	volatile uint32_t AHBENR;      /* Enable AHB,     							Address offset: 0x14 */
	volatile uint32_t APB2ENR;      /* Enable APB2,     							Address offset: 0x18 */
	volatile uint32_t APB1ENR;     /* Enable APB1,                                Address offset: 0x1C */
	volatile uint32_t BDCR;      /* RTC domain control register,     				Address offset: 0x20 */
	volatile uint32_t CSR;      /* Control/status register,     					Address offset: 0x24 */
	volatile uint32_t AHBRSTR;  /*Reset AHB,                                      Address offset: 0x28 */
	volatile uint32_t CFGR2;       /* Clock config register 2,    				Address offset: 0x2C */
	volatile uint32_t CFGR3;       /* Clock config register 3,     				Address offset: 0x30 */
	volatile uint32_t CR2;       /* Clock control register 2,     				Address offset: 0x34 */
}RCC_RegStruct_t;

/*pointer*/
#define RCC 				((RCC_RegStruct_t*)RCC_BASE)

/********************EXTI***********************/
//Extended interrupts and events controller. Between some interrupt sources (GPIOs...) and the NVIC

/*struct*/
typedef struct
{
	volatile uint32_t IMR;    /* Interrupt mask register,          	  	    Address offset: 0x00 */
	volatile uint32_t EMR;    /* Event mask register,              Address offset: 0x04 (due to 4 bytes uint32_t)*/
	volatile uint32_t RTSR;   /* Rising trigger selection,  					Address offset: 0x08 */
	volatile uint32_t FTSR;   /* Falling trigger selection, 					Address offset: 0x0C */
	volatile uint32_t SWIER;  /* SW interrupt event,  						Address offset: 0x10 */
	volatile uint32_t PR;     /* Pending register (write to clear it!),          Address offset: 0x14 */
}EXTI_RegStruct_t;

/*pointer*/
#define EXTI				((EXTI_RegStruct_t*)EXTI_BASE)

/************************GPIO*******************/
/*struct*/
typedef struct
{
	volatile uint32_t MODER;			/* mode(input,general purp,AF,analog)         	Address offset: 0x00 */
	volatile uint32_t OTYPER;           /* output(push-pull,open-drain)     					Address offset: 0x04*/
	volatile uint32_t OSPEEDR;			/* output speed							         	Address offset: 0x08 */
	volatile uint32_t PUPDR;			/* pull-up/down							         	Address offset: 0x0C */
	volatile uint32_t IDR;				/* input register							         	Address offset: 0x10 */
	volatile uint32_t ODR;				/* output register							        Address offset: 0x14 */
	volatile uint32_t BSRR;				/* set/reset						         			Address offset: 0x18 */
	volatile uint32_t LCKR;				/* lock							         			Address offset: 0x1C */
	volatile uint32_t AFR[2];	/* AFR[0]:Altern.funct. low register, AF[1]:AF high    		Address offset: 0x20-0x24 */
	volatile uint32_t BRR;				/* bit reset register							        Address offset: 0x28 */
}GPIO_RegStruct_t;

/*pointers*/
#define GPIOA  				((GPIO_RegStruct_t*)GPIOA_BASE)
#define GPIOB  				((GPIO_RegStruct_t*)GPIOB_BASE)
#define GPIOC  				((GPIO_RegStruct_t*)GPIOC_BASE)
#define GPIOD  				((GPIO_RegStruct_t*)GPIOD_BASE)
#define GPIOE  				((GPIO_RegStruct_t*)GPIOE_BASE)
#define GPIOF  				((GPIO_RegStruct_t*)GPIOF_BASE)

/*enable GPIO port peripheral clocks*/
#define GPIOA_EnableClock()		((RCC->AHBENR)|=(1<<17))
#define GPIOB_EnableClock()		((RCC->AHBENR)|=(1<<18))
#define GPIOC_EnableClock()		((RCC->AHBENR)|=(1<<19))
#define GPIOD_EnableClock()		((RCC->AHBENR)|=(1<<20))
#define GPIOE_EnableClock()		((RCC->AHBENR)|=(1<<21))
#define GPIOF_EnableClock()		((RCC->AHBENR)|=(1<<22))

/*disable GPIO port peripheral clocks*/
#define GPIOA_DisableClock()	((RCC->AHBENR)&= (~(1<<17)))
#define GPIOB_DisableClock()	((RCC->AHBENR)&= (~(1<<18)))
#define GPIOC_DisableClock()	((RCC->AHBENR)&= (~(1<<19)))
#define GPIOD_DisableClock()	((RCC->AHBENR)&= (~(1<<20)))
#define GPIOE_DisableClock()	((RCC->AHBENR)&= (~(1<<21)))
#define GPIOF_DisableClock()	((RCC->AHBENR)&= (~(1<<22)))

/*reset GPIO port*/
#define GPIOA_Reset()	((RCC->AHBRSTR)|=(1<<17))
#define GPIOB_Reset()	((RCC->AHBRSTR)|=(1<<18))
#define GPIOC_Reset()	((RCC->AHBRSTR)|=(1<<19))
#define GPIOD_Reset()	((RCC->AHBRSTR)|=(1<<20))
#define GPIOE_Reset()	((RCC->AHBRSTR)|=(1<<21))
#define GPIOF_Reset()	((RCC->AHBRSTR)|=(1<<22))

/*linking GPIO port to a number, useful for EXTICR bits configuration*/
#define GPIO_PORT_TO_NUMBER(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:0)

/*************************SYSTEM CONTROL*****************************/
//struct
typedef struct
{
	volatile uint32_t CFGR1;       /* configuration 1                  Address offset: 0x00      */
	volatile uint32_t RESERVED1;          /*  									  Address offset: 0x04      */
	volatile uint32_t EXTICR[4];	/*4 external interrupt  config registers 	Address offset: 0x08-0x14 */
	volatile uint32_t CFGR2;    /*configuration 2							Address offset: 0x18 */
	/*CREATE ITLINEs in another struct? they start at offset 0x80!*/
} SYSCFG_RegStruct_t;

//pointer
#define SYSCFG				((SYSCFG_RegStruct_t*)SYSCFG_COMP_BASE)

//enable clock for SYSCFG
#define SYSCFG_EnableClock()		((RCC->APB2ENR) |= (1<<0))	/*SYSCFGCOMP is bit 0 in APB2ENR, which is in RCC */

/*************************SPI/I2S(not I2C!! I2S is sound related)*****************************/
//struct referred to registers
typedef struct
{
	volatile uint32_t SPI_CR1;       /*control register 1                  Address offset: 0x00      */
	volatile uint32_t SPI_CR2;       /*control register 2  					Address offset: 0x04      */
	volatile uint32_t SPI_SR;       /*status register  						Address offset: 0x08 */
	volatile uint32_t SPI_DR;       /*data register 						Address offset: 0x0C */
	volatile uint32_t SPI_CRCPR;       /*CRC polyn. register 				Address offset: 0x10 */
	volatile uint32_t SPI_RXCRC;       /*RX CRC register 				Address offset: 0x14 */
	volatile uint32_t SPI_TXCRC;       /*TX CRC register 				Address offset: 0x18 */
	volatile uint32_t SPI_I2S;       /*SPI(0) or I2S(1) 				Address offset: 0x1C */
	volatile uint32_t SPI_I2S_PR;       /*Prescaler register 			Address offset: 0x20 */

} SPI_RegStruct_t;

//pointer
#define SPI1_I2S1				((SPI_RegStruct_t*)SPI1_I2S1_BASE)
#define SPI2					((SPI_RegStruct_t*)SPI2_BASE)

//enable SPI peripheral clock
#define SPI1_I2S1_EnableClock()		((RCC->APB2ENR)|=(1<<12))
#define SPI2_EnableClock()			((RCC->APB1ENR)|=(1<<14))

//disable SPI peripheral clock
#define SPI1_I2S1_DisableClock()		((RCC->APB2ENR)&=(~(1<<12)))
#define SPI2_DisableClock()			((RCC->APB1ENR)&=(~(1<<14)))

//Reset SPI peripheral clock
#define SPI1_ResetClock()		((RCC->APB2RSTR)|=(1<<12))
#define SPI2_ResetClock()			((RCC->APB1RSTR)|=(1<<14))

//Bits of the SPI registers
//SPI_CR1
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_CRCL     			 	11 //Cyclic Redundancy Check, not data size (this is DS in CR2)
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

//Bit position definitions SPI_CR2
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_NSSP				 	3
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7
#define SPI_CR2_DS						8
#define SPI_CR2_FRXTH					12
#define SPI_CR2_LDMA_RX					13
#define SPI_CR2_LDMA_TX					14

//Bit position definitions SPI_SR
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8
#define SPI_SR_FRLVL					9
#define SPI_SR_FTLVL				 	11

/*************************I2C*******************************************************************************/
//struct referred to registers
typedef struct
{
	volatile uint32_t I2C_CR1;      	/*control register 1               	Address offset: 0x00 */
	volatile uint32_t I2C_CR2;       	/*control register 2  				Address offset: 0x04 */
	volatile uint32_t I2C_OAR1;       	/*own address 1 register  			Address offset: 0x08 */
	volatile uint32_t I2C_OAR2;       	/*own address 2 register 			Address offset: 0x0C */
	volatile uint32_t I2C_TIMINGR;    	/*Timing register 					Address offset: 0x10 */
	volatile uint32_t I2C_TIMEOUTR;   	/*Timeout register 					Address offset: 0x14 */
	volatile uint32_t I2C_ISR;       	/*Interrupt and status register 	Address offset: 0x18 */
	volatile uint32_t I2C_ICR;       	/*Interrupt clear register 			Address offset: 0x1C */
	volatile uint32_t I2C_PECR;       	/*PEC register 						Address offset: 0x20 */
	volatile uint32_t I2C_RXDR;       	/*Receive data register 			Address offset: 0x24 */
	volatile uint32_t I2C_TXDR;       	/*Transmit data register 			Address offset: 0x28 */
} I2C_RegStruct_t;

//pointer
#define I2C1						((I2C_RegStruct_t*)I2C1_BASE)
#define I2C2						((I2C_RegStruct_t*)I2C2_BASE)

//enable I2C peripheral clock
#define I2C1_EnableClock()			((RCC->APB1ENR)|=(1<<21))
#define I2C2_EnableClock()			((RCC->APB1ENR)|=(1<<22))

//disable I2C peripheral clock
#define I2C1_DisableClock()			((RCC->APB1ENR)&=(~(1<<21)))
#define I2C2_DisableClock()			((RCC->APB1ENR)&=(~(1<<22)))

//Reset I2C peripheral clock
#define I2C1_ResetClock()			((RCC->APB1RSTR)|=(1<<21))
#define I2C2_ResetClock()			((RCC->APB1RSTR)|=(1<<22))

//Bits of the I2C registers
//I2C_CR1
#define I2C_CR1_PE	     				 0
#define I2C_CR1_TXIE      				 1
#define I2C_CR1_RXIE     				 2
#define I2C_CR1_ADDRIE   				 3
#define I2C_CR1_NACKIE     				 4
#define I2C_CR1_STOPIE   			 	 5
#define I2C_CR1_TCIE     				 6
#define I2C_CR1_ERRIE     				 7
#define I2C_CR1_DNF      		 		 8//[3:0]
#define I2C_CR1_ANFOFF     			 	 12
#define I2C_CR1_CRCNEXT   			 	 13
#define I2C_CR1_TXDMAEN   			 	 14
#define I2C_CR1_RXDMAEN     			 15
#define I2C_CR1_SBC      				 16
#define I2C_CR1_NOSTRETCH      			 17
#define I2C_CR1_WUPEN      				 18
#define I2C_CR1_GCEN      				 19
#define I2C_CR1_SMBHEN      			 20
#define I2C_CR1_SMBDEN      			 21
#define I2C_CR1_ALERTEN      			 22
#define I2C_CR1_PECEN      				 23

//Bit position definitions I2C_CR2
#define I2C_CR2_SADD		 			0 //[9:0]
#define I2C_CR2_RDWRN				 	10
#define I2C_CR2_ADD10				 	11
#define I2C_CR2_HEAD10R				 	12
#define I2C_CR2_START					13
#define I2C_CR2_STOP					14
#define I2C_CR2_NACK				 	15
#define I2C_CR2_NBYTES					16 //[7:0]
#define I2C_CR2_RELOAD					24
#define I2C_CR2_AUTOEND					25
#define I2C_CR2_PECBYTE					26

//Bit position definitions I2C_OAR1
#define I2C_OAR1_OA1					0 //[9:0]
#define I2C_OAR1_OA1MODE				10
#define I2C_OAR1_OA1EN				 	15

//Bit position definitions I2C_OAR2
#define I2C_OAR2_OA2					1 //[7:1]
#define I2C_OAR2_OA2MSK					8 //[2:0]
#define I2C_OAR2_OA2EN				 	15

//Bit position definitions I2C_TIMINGR
#define I2C_TIMINGR_SCLL					0 //[7:0]
#define I2C_TIMINGR_SCLH					8 //[7:0]
#define I2C_TIMINGR_SDADEL				 	16 //[3:0]
#define I2C_TIMINGR_SCLDEL				 	20 //[3:0]
#define I2C_TIMINGR_PRESC				 	28 //[3:0]

//Bit position definitions I2C_TIMEOUTR
#define I2C_TIMEOUTR_TIMEOUTA				0 //[11:0]
#define I2C_TIMEOUTR_TIDLE					12
#define I2C_TIMEOUTR_TIMEOUTEN				15
#define I2C_TIMEOUTR_TIMEOUTB			 	16 //[11:0]
#define I2C_TIMEOUTR_TEXTEN				 	31

//Bit position definitions I2C_ISR
#define I2C_ISR_TXE		 			0 //[9:0]
#define I2C_ISR_TXIS				1
#define I2C_ISR_RXNE				2
#define I2C_ISR_ADDR				3
#define I2C_ISR_NACKF				4
#define I2C_ISR_STOPF				5
#define I2C_ISR_TC				 	6
#define I2C_ISR_TCR					7
#define I2C_ISR_BERR				8
#define I2C_ISR_ARLO				9
#define I2C_ISR_OVR					10
#define I2C_ISR_PECERR				11
#define I2C_ISR_TIMEOUT				12
#define I2C_ISR_ALERT				13
#define I2C_ISR_BUSY				15
#define I2C_ISR_DIR					16
#define I2C_ISR_ADDCODE				17 //[6:0]

//Bit position definitions I2C_ICR
#define I2C_ICR_ADDRCF		 		3
#define I2C_ICR_NACKCF				4
#define I2C_ICR_STOPCF				5
#define I2C_ICR_BERRCF				8
#define I2C_ICR_ARLOCF				9
#define I2C_ICR_OVRCF				10
#define I2C_ICR_PECCF				11
#define I2C_ICR_TIMEOUTCF			12
#define I2C_ICR_ALERTCF				13

//Bit position definitions I2C_PECR
#define I2C_PECR_PEC		 		0 //[7:0]

//Bit position definitions I2C_RXDR
#define I2C_RXDR_RXDATA		 		0 //[7:0]

//Bit position definitions I2C_TXDR
#define I2C_TXDR_TXDATA		 		0 //[7:0]

//Adress mode
#define I2C_ADDRESS_MODE_7BIT		0
#define I2C_ADDRESS_MODE_10BIT		1

/*************************************BASIC TIMERS****************************************************************/
//struct referred to registers.16-bit addressable registers,but offsets are every 32bits (+gap between CR2-DIER and EGR-CNT)
typedef struct{
	volatile uint32_t TIM_CR1; /*control register 1.offset:0x00*/
	volatile uint32_t TIM_CR2; /*control register 2.offset:0x04*/
	volatile uint32_t EMPTY1; /*0x08*/
	volatile uint32_t TIM_DIER; /*DMA/interrupt enable.offset:0x0C*/
	volatile uint32_t TIM_SR; /*status register (UIF:update interrupt flag).offset:0x10*/
	volatile uint32_t TIM_EGR; /*event generation register* (UG:update generation reinitializes counter).offset:0x14*/
	volatile uint32_t EMPTY2; /*offset:0x18*/
	volatile uint32_t EMPTY3; /*offset:0x1C*/
	volatile uint32_t EMPTY4; /*offset:0x20*/
	volatile uint32_t TIM_CNT; /*counter. offset: 0x24*/
	volatile uint32_t TIM_PSC; /*prescaler PSC: fCK_PSC / (PSC[15:0] + 1). offset: 0x28*/
	volatile uint32_t TIM_ARR; /*it's the value to be loaded into autoreload. offset: 0x2C*/
}BasicTimer_RegStruct_t;

//pointer
#define BASIC_TIMER_6 ((BasicTimer_RegStruct_t*)TIM6_BASE)
#define BASIC_TIMER_7 ((BasicTimer_RegStruct_t*)TIM7_BASE)

//enable timer peripheral clock
#define BasicTimer_6_Clock_Enable()			((RCC->APB1ENR)|=(1<<4))
#define BasicTimer_7_Clock_Enable()			((RCC->APB1ENR)|=(1<<5))

//disable timer peripheral clock
#define BasicTimer_6_Clock_Disable()			((RCC->APB1ENR)&=(~(1<<4)))
#define BasicTimer_7_Clock_Disable()			((RCC->APB1ENR)&=(~(1<<5)))

//Reset timer peripheral clock
#define BasicTimer_6_Clock_Reset()			((RCC->APB1RSTR)|=(1<<4))
#define BasicTimer_7_Clock_Reset()			((RCC->APB1RSTR)|=(1<<5))

//bit positions in the registers
//CR1
#define TIM_CR1_ARPE 7 //autoreload preload enable; 0: not buffered; 1: buffered
#define TIM_CR1_OPM 3 //one-pulse mode; 0: counter not stopped at update event
#define TIM_CR1_URS 2 //update request source; 1: only overflow/underflow generates interrupt/DMA; 0: all the possible sources can generate it
#define TIM_CR1_UDIS 1 //update disable; 1: UEV (update event) disabled
#define TIM_CR1_CEN 0 //counter enabled if 1
//CR2
#define TIM_CR2_MMS 4 //[6:4] master mode selection
//DIER
#define TIM_DIER_UDE 8 //Update DMA request enable if 1
#define TIM_DIER_UIE 0 //Update interrupt enable if 1
//SR
#define TIM_SR_UIF 0 //set by HW on an update event (overflow/underflow if UDIS=0 ; or when counter reinitialized by TIM_EGR_UG if URS,UDIS=0)
//EGR
#define TIM_EGR_UG 0 //update generation. if 1, counter reinitializes
//CNT
#define TIM_CNT_CNT 0 //[15:0]
//PSC
#define TIM_PSC_PSC //[15:0]
//ARR
#define TIM_ARR_ARR //[15:0]

/*************INCLUDE DRIVER HEADERS************/
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "basicTimers.h"

#endif /* INCDRIVERS_STM32F091RCT6_H_ */
