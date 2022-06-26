/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body #2 (LEDs+Buttons+SPI with Arduino as slave and just printing)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <string.h> //because I want to use memset to initialize the handles to 0
#include <stdint.h>
#include <stdio.h>
#include "stm32f091rct6.h"

/*to debug with printf */
//extern void initialise_monitor_handles(void);

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*2 buttons causing interruptions: OnOff(PB3) and Emergency(PB5). 4 LEDs */

//GPIO_PinHandle_t butOnOff, butEmerg, led0, led1, led2, led3;
GPIO_PinHandle_t buttons, leds,spiGPIOs;
#define PIN_LED0 0
#define PIN_LED1 1
#define PIN_LED2 9 //PA2 TX? it seems not to work when debugging
#define PIN_LED3 10 //PA3 RX? it seems not to work when debugging
#define PIN_BUTTON_OFFON 3
#define PIN_BUTTON_EMERGENCY 5

/*SPI handle*/
SPI_Handle_t spi1;
#define PIN_SPI1_NSS 4
#define PIN_SPI1_SCK 5
#define PIN_SPI1_MISO 6
#define PIN_SPI1_MOSI 7

enum state{OFF,ON,EMERGENCY};
enum state currentState = OFF;
uint8_t stateChanged = 0; //0 if the state changes; 1 if it changes

/*SPI communication with an Arduino (aka Ard).Whenever state changes, send it*/
//Commands
#define SPI_COMMAND_PRINT  	0x50
#define SPI_COMMAND_LED		0x51
/*Notes:in SPI comm slave will not initiate the data transfer.We send a command to Ard,Ard reads it,puts ACK/NACK.
*When the command is sent to Ard,Ard RX buffer saves this and puts an ACK/NACK in its TX.During command transmission,
*master has received garbage bytes from Ard.So before requesting ACK/NACK,we need to clear off master RX by reading.Now we can get
*Ard's ACK/NACK by sending dummy data*/
#define SPI_ARD_ACK		0xA1
#define SPI_ARD_NACK	0xB1

uint8_t myTXbuffer[2]; //2 will be maximum size. we could try with dynamic arrays based on int*
uint8_t myRXbuffer[2]; //we could use one single buffer for both TX and RX if we had scarce memory
uint8_t dummy_byte = 0xC1; //dummy byte for SPI

int stringLength(char *charSt){ //to know the length of a string instead of using external functions
	int length = 0;
	char *pointer = charSt; //to avoid modifying charSt
	while (*pointer!='\0'){ //'\0'end
		length++;
		pointer++;
	}
	return length;
}

void delay(void)
{
	// some delay for the debouncing of the buttons
	for(uint32_t i = 0 ; i < 300000/2 ; i ++);
}

//Configurations and initializations
void peripheral_Config_Ini(void);
/**********************************************START MAIN********************************************************************/
int main(void)
{
	//initialise_monitor_handles(); //for debugging in IDE
	//printf("THIS STARTS\n");


	//initialize structures to 0 to avoid garbage values
	memset(&buttons,0,sizeof(buttons));
	memset(&leds,0,sizeof(leds));

	/*GPIOs for the SPI handled by spiGPIO -->SPI1 PA4:NSS PA4:SCK PA4:MISO PA4:MOSI*/
	GPIO_PinHandle_t spiGPIOs;
	memset(&spiGPIOs,0,sizeof(spiGPIOs));
	memset(&spi1,0,sizeof(spi1));

	//Configurations and initilizations
	peripheral_Config_Ini();

	/*configure interrupts (without priorities atm)*/
	//Button OnOff (GPIOB0)
	GPIO_IRQ_EnableDisable(IRQ_EXTI2_3, ENABLE);
	//Button Emergency (GPIOB2)
	GPIO_IRQ_EnableDisable(IRQ_EXTI4_15, ENABLE);
	//SPI1
	SPI_IRQ_EnableDisable(IRQ_SPI1, ENABLE);

	/* Loop forever */
	while (1){
		//if state changes, send print command and the content. Think about a way to do it with interrupt due to value change
		if (stateChanged) {
			//enable SPI and disable it at the end?

			//Send print command.Ard should store it in its RX buffer and put ACK in TX.Master should receive garbage in shift reg.
			myTXbuffer[0] = SPI_COMMAND_PRINT;
			SPI_Send(&spi1, myTXbuffer, 1);
			//SPI_Send(&spi1, (uint8_t*)SPI_COMMAND_PRINT, 1);
			while (spi1.SPI_Comm.TX_state != SPI_READY);

			//Read master RX to clear it off for the next reading
			SPI_Read(&spi1, myRXbuffer, 1); //1 byte to be read
			while (spi1.SPI_Comm.RX_state != SPI_READY);

			//Send dummy data to Ard so that master receives ACK/NACK
			SPI_Send(&spi1, &dummy_byte, 1); //length 1 byte
			while (spi1.SPI_Comm.TX_state != SPI_READY);

			//Read master RX to get ACK/NACK
			SPI_Read(&spi1, myRXbuffer, 1); //1 byte to be read
			while (spi1.SPI_Comm.RX_state != SPI_READY);
			//printf("myRXbuffer ACK: %s/n", myRXbuffer);

			//if ACK, send length+currentState and receive response
			if (*myRXbuffer == SPI_ARD_ACK){
				myTXbuffer[0] = 8; //in bits
				myTXbuffer[1] = currentState;
				SPI_Send(&spi1, myTXbuffer, 2); //length is 2
				while (spi1.SPI_Comm.TX_state != SPI_READY);

				SPI_Read(&spi1, myRXbuffer, 2); //2 bytes to be read (dummy read).
				while (spi1.SPI_Comm.RX_state != SPI_READY);
				//printf("myRXbuffer after sending state: %s/n", myRXbuffer);

				//myTXbuffer = "statusChanged: ";
				//printf("status changed to: %d/n",currentState);
			}
			else{
				//printf("no ACK received/n");
			}

			//clear the flag of current state in order to capture state changes
			stateChanged = 0;

			//Disable SPI?
		}

	}
}
/**********************************************END MAIN********************************************************************/

/***********************************************ISR handlers****************************************************************/
void EXTI2_3_IRQHandler(void){ //when button on-off
	delay();
	GPIO_IRQ_Handling(PIN_BUTTON_OFFON);
	if (currentState == OFF){
		currentState = ON;
		stateChanged = 1;
		GPIO_WritePin(GPIOA,PIN_LED0,0);
		GPIO_WritePin(GPIOA,PIN_LED1,1);
		GPIO_WritePin(GPIOA,PIN_LED2,1);
		GPIO_WritePin(GPIOA,PIN_LED3,0);
	}
	else{
		currentState = OFF;
		stateChanged = 1;
		GPIO_WritePin(GPIOA,PIN_LED0,0);
		GPIO_WritePin(GPIOA,PIN_LED1,0);
		GPIO_WritePin(GPIOA,PIN_LED2,0);
		GPIO_WritePin(GPIOA,PIN_LED3,0);
	}
	//printf("%d\n",currentState);
}

void EXTI4_15_IRQHandler(void){
	delay();
	GPIO_IRQ_Handling(PIN_BUTTON_EMERGENCY);
	if (currentState != EMERGENCY){
		stateChanged = 1;
	}
	currentState = EMERGENCY;
	//printf("%d\n",currentState);
	GPIO_WritePin(GPIOA,PIN_LED0,1);
	GPIO_WritePin(GPIOA,PIN_LED1,1);
	GPIO_WritePin(GPIOA,PIN_LED2,1);
	GPIO_WritePin(GPIOA,PIN_LED3,1);

}

//SPI IRQ handler for SPI1
void SPI1_IRQHandler(void){
	SPI_IRQ_Handling(&spi1);
}

/**********************************************END IRQ********************************************************************/

/*****************************Configurations and initializations************************************************************/
void peripheral_Config_Ini(void){

	//LED0
	leds.pGPIO = GPIOA;
	leds.GPIO_PinConfig.GPIO_PinNumber = PIN_LED0;
	leds.GPIO_PinConfig.GPIO_PinMode = GPIO_OUT;
	leds.GPIO_PinConfig.GPIO_PinOutType = GPIO_PUSHPULL;
	leds.GPIO_PinConfig.GPIO_PinOutSpeed = GPIO_SLOWSPEED; //GPIO_MEDIUMSPEED
	leds.GPIO_PinConfig.GPIO_PinPullUpDown = GPIO_NOPULL;
	//Initialization LED0
	GPIO_PinInit(&leds);

	//LED1.Port,Mode,OutType,OutSpeed and PullUpDown same as LED0
	leds.GPIO_PinConfig.GPIO_PinNumber = PIN_LED1;
	//Initialization LED1
	GPIO_PinInit(&leds);

	//LED2.Port,Mode,OutType,OutSpeed and PullUpDown same as LED0
	leds.GPIO_PinConfig.GPIO_PinNumber = PIN_LED2;
	//Initialization LED2
	GPIO_PinInit(&leds);

	//LED3.Port,Mode,OutType,OutSpeed and PullUpDown same as LED0
	leds.GPIO_PinConfig.GPIO_PinNumber = PIN_LED3;
	//Initialization LED3
	GPIO_PinInit(&leds);

	//LEDs OFF
	GPIO_WritePin(GPIOA,PIN_LED0,1);
	GPIO_WritePin(GPIOA,PIN_LED1,0);
	GPIO_WritePin(GPIOA,PIN_LED2,0);
	GPIO_WritePin(GPIOA,PIN_LED3,1);

	//BUTTON ON-OFF
	buttons.pGPIO = GPIOB;
	buttons.GPIO_PinConfig.GPIO_PinNumber = PIN_BUTTON_OFFON;
	buttons.GPIO_PinConfig.GPIO_PinMode = GPIO_FALL_TRIG; //interrupt falling edge
	buttons.GPIO_PinConfig.GPIO_PinPullUpDown = GPIO_PULLUP; //pull up (3.3 by default)
	//initialization button on-off
	GPIO_PinInit(&buttons);

	//EMERGENCY BUTTON
	buttons.pGPIO = GPIOB;
	buttons.GPIO_PinConfig.GPIO_PinNumber = PIN_BUTTON_EMERGENCY;
	buttons.GPIO_PinConfig.GPIO_PinMode = GPIO_FALL_TRIG;
	buttons.GPIO_PinConfig.GPIO_PinPullUpDown = GPIO_PULLUP;
	//initialization emergency button
	GPIO_PinInit(&buttons);

	//GPIOs for SPI: SPI1 PA4:NSS PA5:SCK PA6:MISO PA7:MOSI
	spiGPIOs.pGPIO = GPIOA;
	spiGPIOs.GPIO_PinConfig.GPIO_PinMode = GPIO_ALTFUN;
	spiGPIOs.GPIO_PinConfig.GPIO_PinOutType = GPIO_PUSHPULL;
	spiGPIOs.GPIO_PinConfig.GPIO_PinOutSpeed = GPIO_HIGHSPEED; //FAST SPEED
	spiGPIOs.GPIO_PinConfig.GPIO_PinPullUpDown = GPIO_NOPULL;
	//NSS
	spiGPIOs.GPIO_PinConfig.GPIO_PinNumber = PIN_SPI1_NSS;//NSS
	GPIO_PinInit(&spiGPIOs);
	//SCK
	spiGPIOs.GPIO_PinConfig.GPIO_PinNumber = PIN_SPI1_SCK;//SCK
	GPIO_PinInit(&spiGPIOs);
	//MISO
	spiGPIOs.GPIO_PinConfig.GPIO_PinNumber = PIN_SPI1_MISO;//MISO
	GPIO_PinInit(&spiGPIOs);
	//MOSI
	spiGPIOs.GPIO_PinConfig.GPIO_PinNumber = PIN_SPI1_MOSI;//MOSI
	GPIO_PinInit(&spiGPIOs);

	/*SPI handle configuration and initialization*/
	//SPI configuration
	spi1.pSPI = SPI1_I2S1;
	spi1.SPI_Config.SPI_Mode = SPI_MASTER;
	spi1.SPI_Config.SPI_Speed = SPI_SPEED_2;
	spi1.SPI_Config.SPI_CommType = SPI_FULLDUPLEX;
	spi1.SPI_Config.SPI_DataSize = SPI_8BIT;
	spi1.SPI_Config.SPI_SWslave = SPI_HW_MGMT;
	spi1.SPI_Config.SPI_Pol = SPI_CLK_IDLE_0;
	spi1.SPI_Config.SPI_Phase = SPI_CLK_CAPT_FIRST;

	//SPI initialization
	SPI_Init(&spi1);

	//Set SSOE to 1 to make NSS output enable
	SPI_SSOE(&spi1,ENABLE);

	//Enable SPI peripheral
	SPI_EnableDisable(&spi1,ENABLE);
}

