/*
 * gpio.h
 *
 *  Created on: May 19, 2022
 *      Author: danim
 */

#ifndef INCDRIVERS_GPIO_H_
#define INCDRIVERS_GPIO_H_

#include "stm32f091rct6.h"
/*************************************************STRUCTS***********************************************************/
/*structure to configure a GPIO pin, which will be linked to the registers(GPIO_RegStruct_t in stm32f091rct6.h) in the driver functions*/
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOutType;
	uint8_t GPIO_PinOutSpeed;
	uint8_t GPIO_PinPullUpDown;
	uint8_t GPIO_PinAlterFunc;
}GPIO_PinConfig_t;

/*structure to handle a GPIO pin: configuration struct () + pointer to register struct (GPIO_RegStruct_t in stm32f091rct6.h)*/
typedef struct{
	GPIO_RegStruct_t *pGPIO; //Points to address of GPIO port,so it will be GPIOA,GPIOB..GPIOF (macros in stm32f091rct6.h)
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_PinHandle_t;

/*************************************************MACROS***********************************************************/
//Modes. 0-3 for MODER register; 4-6 for us to handle interrupts
#define GPIO_IN 		0
#define GPIO_OUT 		1
#define GPIO_ALTFUN 	2
#define GPIO_ANA 		3
#define GPIO_FALL_TRIG     4
#define GPIO_RISE_TRIG     5
#define GPIO_FALLRISE_TRIG    6

//output type
#define GPIO_PUSHPULL		0
#define GPIO_OPENDRAIN		1

//output speed
#define GPIO_SLOWSPEED		0
#define GPIO_MEDIUMSPEED	1
#define GPIO_HIGHSPEED		3

//Pull up/down
#define GPIO_NOPULL 		0
#define GPIO_PULLUP 		1
#define GPIO_PULLDOWN 		2
#define GPIO_RESERVEDPULL	3

//Alternate functions
#define GPIO_AF0	0
#define GPIO_AF1	1
#define GPIO_AF2	2
#define GPIO_AF3	3
#define GPIO_AF4	4
#define GPIO_AF5	5
#define GPIO_AF6	6
#define GPIO_AF7	7
/*********************************Prototypes of APIs (functions) defined in gpio.c***********************************/
/*Enable or disable GPIO port peripheral clock*/
void GPIO_ClockControl(GPIO_RegStruct_t *pGPIO, uint8_t EnableDisable);

/*Initialize GPIO pin: mode,out type&speed,pull up/down, alter function*/
void GPIO_PinInit(GPIO_PinHandle_t *pGPIO_PinHandle);

/*Reset GPIO port (not only pin!)*/
void GPIO_ResetPort(GPIO_RegStruct_t *pGPIO);

/*Read from input (pin or port)*/
uint8_t GPIO_ReadPin(GPIO_RegStruct_t *pGPIO, uint8_t pin);
uint16_t GPIO_ReadPort(GPIO_RegStruct_t *pGPIO);

/*Write to output (pin or port)*/
void GPIO_WritePin(GPIO_RegStruct_t *pGPIO, uint8_t pin, uint8_t output);
void GPIO_WritePort(GPIO_RegStruct_t *pGPIO, uint16_t output);
void GPIO_TogglePin(GPIO_RegStruct_t *pGPIO, uint8_t pin);
//toggle port doesn't seem to make sense

/*Lock pin*/
//to be defined. lock the configuration of the port bits when a correct write sequence

/*IRQ Configuration and ISR handling*/
void GPIO_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable);
void GPIO_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void GPIO_IRQ_Handling(uint8_t pin);


#endif /* INCDRIVERS_GPIO_H_ */
