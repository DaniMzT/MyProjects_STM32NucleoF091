/*
 * spi.h
 *
 *  Created on: 2 jun. 2022
 *      Author: danim
 */

#ifndef INCDRIVERS_SPI_H_
#define INCDRIVERS_SPI_H_

#include "stm32f091rct6.h"

/***********************************STRUCTS****************************************************/
/*structure to configure SPI, which will be linked to the registers(SPI_RegStruct_t in stm32f091rct6.h) in the driver functions*/
typedef struct{
	uint8_t SPI_Mode; //master/slave
	uint8_t SPI_Speed; //clock speed
	uint8_t SPI_CommType; //full duplex, half duplex or simplex
	uint8_t SPI_DataSize; //data frame format: 8 or 16 bits?
	uint8_t SPI_SWslave; //SSM: software slave management. if 1, SSI (bit 8 CR1) controls NNS pin
	uint8_t SPI_Pol; //Polarity: when idle state(no data transfer):if 1,no data while HIGH;if 0, no data while LOW
	uint8_t SPI_Phase; //Phase: at which edge (first or second) data is sampled (captured)
}SPI_Config_t;

/*structure used during communication (basically during interrupts) */
typedef struct{
	uint32_t TX_length; //length buffer TX in bits. Every char is 1 byte
	uint8_t *TX_buffer; //not the buffer register, but a variable where we put our data to be sent to DR
	uint8_t TX_state;
	uint32_t RX_length;
	uint8_t *RX_buffer; //not the buffer register, but a variable where we will put the data to be read from DR
	uint8_t RX_state;
}SPI_Comm_t;

/*structure to handle SPI: configuration struct + pointer to registers(SPI_RegStruct_t in stm32..h) + communication struct*/
typedef struct{
	SPI_RegStruct_t *pSPI; //Points to SPI registers,so it will be SPI1 or SPI2 (macros in stm32f091rct6.h)
	SPI_Config_t SPI_Config; //configuration struct
	SPI_Comm_t SPI_Comm; //communication struct
}SPI_Handle_t;

/************************************************Macros*******************************************************/

//Device mode
#define SPI_SLAVE     0
#define SPI_MASTER    1

//SPI clock speed divider
#define SPI_SPEED_2             	0
#define SPI_SPEED_4             	1
#define SPI_SPEED_8             	2
#define SPI_SPEED_16             	3
#define SPI_SPEED_32             	4
#define SPI_SPEED_64             	5
#define SPI_SPEED_128             	6
#define SPI_SPEED_256             	7

//Bus configuration type
#define SPI_FULLDUPLEX              1
#define SPI_HALFDUPLEX              2
#define SPI_SIMPLEX_RXONLY    		3
//simplex TX is actually full duplex

//data size
#define SPI_4BIT	3
#define SPI_5BIT	4
#define SPI_6BIT	5
#define SPI_7BIT	6
#define SPI_8BIT	7
#define SPI_9BIT	8
#define SPI_10BIT	9
#define SPI_11BIT	10
#define SPI_12BIT	11
#define SPI_13BIT	12
#define SPI_14BIT	13
#define SPI_15BIT	14
#define SPI_16BIT	15

//Polarity
#define SPI_CLK_IDLE_0	0
#define SPI_CLK_IDLE_1	1

//Phase
#define SPI_CLK_CAPT_FIRST	0
#define SPI_CLK_CAPT_SECOND	1

//HW/SW slave management
#define SPI_HW_MGMT	0
#define SPI_SW_MGMT	1

//Events for the callback, related to interrupts
#define SPI_TX_FINISHED 0
#define SPI_RX_FINISHED 1
#define SPI_OVR_EVENT 	2

//state
#define SPI_READY 		0
#define SPI_DURING_TX	1
#define SPI_DURING_RX	1

/*********************************************APIs************************************************************/

/*Enable or disable SPI peripheral clock*/
void SPI_ClockControl(SPI_Handle_t *pSPIhandle, uint8_t EnableDisable);

/*Initialization*/
void SPI_Init(SPI_Handle_t *pSPIhandle);

/*SPI control: enable/disable --> bit 6 CR1 */
void SPI_EnableDisable(SPI_Handle_t *pSPIhandle, uint8_t EnableDisable);

/*Reset SPI */
void SPI_Reset(SPI_Handle_t *pSPIhandle);

/*SSI: internal slave select*/
void SPI_SSI(SPI_Handle_t *pSPIhandle, uint8_t EnableDisable);

/*SSOE: SS output enable*/
void SPI_SSOE(SPI_Handle_t *pSPIhandle, uint8_t EnableDisable);

/*Get flag*/
uint8_t SPI_GetFlagStatus(SPI_Handle_t *pSPIhandle , uint32_t Flag);

/*IRQ Configuration and ISR handling*/
void SPI_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable);
void SPI_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void SPI_IRQ_Handling(SPI_Handle_t *pSPIhandle);

/*Send or receive data (through interrupts, not polling) */
void SPI_Send(SPI_Handle_t *pSPIhandle, uint8_t *pTXbuffer, uint32_t length);
void SPI_Read(SPI_Handle_t *pSPIhandle, volatile uint8_t *pRXbuffer, uint32_t length);

/*Callback to application */
void SPI_App_Callback(SPI_Handle_t *pSPIhandle,uint8_t Event);

/*Clear OVR (overrun) flag. This is just in case that the main application wants to do it on its own */
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIhandle);

#endif /* INCDRIVERS_SPI_H_ */
