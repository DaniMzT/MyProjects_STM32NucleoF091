/*
 * i2c.h
 *
 *  Created on: Oct 2, 2022
 *      Author: danimt
 */

#ifndef INCDRIVERS_I2C_H_
#define INCDRIVERS_I2C_H_
#include "stm32f091rct6.h"

//NOTE: Currently,i2c.h is developed for STM32 as a master, to connect with a slave
/***********************************STRUCTS****************************************************/
/*structure to configure I2C, which will be linked to the registers(SPI_RegStruct_t in stm32f091rct6.h) in the driver functions*/
typedef struct{
	uint8_t I2C_Mode; //master/slave
	uint8_t I2C_AddressMode; //0: 7-bit; 1: 10-bit
	uint32_t I2C_Timing; //clock configuration (TIMINGR register)
}I2C_Config_t;

/*structure used during communication (basically during interrupts) */
typedef struct{
	uint32_t TX_length; //length buffer TX in bytes
	uint8_t *TX_buffer; //not the buffer register, but a variable where we put our data to be put into TXDR
	uint32_t RX_length;
	uint8_t *RX_buffer; //not the buffer register, but a variable where we will put the data to be read from RXDR
	uint8_t communication_state; //TX and RX included here
	uint8_t I2C_RepeatStart; //related to Autoend. for master mode (=1 automatic stop after nbytes transferred; if restart,=0)
	uint8_t I2C_Nbytes; //number of bytes to be transmitted/received
	uint16_t I2C_SlaveAddress; //7-bit or 10-bit
}I2C_Comm_t;

/*structure to handle I2C: configuration struct + pointer to registers(I2C_RegStruct_t in stm32..h) + communication struct*/
typedef struct{
	I2C_RegStruct_t *pI2C; ; //Points to I2C registers,so it will be I2C1 or I2C2 (macros in stm32f091rct6.h)
	I2C_Config_t I2C_Config; //configuration struct
	I2C_Comm_t I2C_Comm_t; //communication struct
}I2C_Handle_t;

/************************************************Macros*******************************************************/
//Events for the callback, related to interrupts
#define I2C_TX_STARTED  0
#define I2C_RX_STARTED 	1
#define I2C_FINISHED 	2
#define I2C_BERR_ERROR 	3
#define I2C_NEW_READING 4
#define I2C_TC			5
#define I2C_TCR			6

//state
#define I2C_READY 			0
#define I2C_DURING_TX		1
#define I2C_DURING_RX		2
#define I2C_WAITING_END		3 //waiting STOP or TC/TCR
#define I2C_RESTART_STOP	4 //after TC, SCL streched low waiting for restart or stop/
#define I2C_RELOAD			5 //after TCR, waiting for reloading NBYTES (total>255)

/*********************************************APIs************************************************************/
//start+address+read/write, stop, transmission, ack/nack
//others to be considered in a future: reload, APIs for 10-bit addresses, slave APIs...

/*Enable or disable I2C peripheral clock*/
void I2C_ClockControl(I2C_Handle_t *pI2Chandle, uint8_t EnableDisable);

/*Initialization*/
void I2C_Init(I2C_Handle_t *pI2Chandle);

/*I2C control: enable/disable */
void I2C_EnableDisable(I2C_Handle_t *pI2Chandle, uint8_t EnableDisable);

/*Reset I2C */
void I2C_Reset(I2C_Handle_t *pI2Chandle);

/*IRQ Configuration and ISR handling*/
void I2C_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable); //IRQ_I2C1(combined with eXTI line23)/IRQ_I2C2
void I2C_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void I2C_IRQ_Handling(I2C_Handle_t *pI2Chandle);

void I2C_Master_Transmitter(I2C_Handle_t *pI2Chandle, uint8_t exp_bytes, uint8_t autoend);
void I2C_Master_Receiver(I2C_Handle_t *pI2Chandle, uint8_t exp_bytes, uint8_t autoend);
void I2C_Master_Manual_Stop(I2C_Handle_t *pI2Chandle); //sends STOP when autoend=0.called in main program
//to be done: same but in case of STM32 being slave

/*Callback to application */
void I2C_App_Callback(I2C_Handle_t *pI2Chandle,uint8_t Event);

#endif /* INCDRIVERS_I2C_H_ */
