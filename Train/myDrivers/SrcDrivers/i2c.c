/*
 * i2c.c
 *
 *  Created on: Oct 2, 2022
 *      Author: danimt
 */

#include "i2c.h"

/*Private functions, only accessed through i2c.c. Basically the interrupt handlers */
static void I2C_TXIS_handler(I2C_Handle_t *pI2Chandle);
static void I2C_RXNE_handler(I2C_Handle_t *pI2Chandle);
static void I2C_BERR_handler(I2C_Handle_t *pI2Chandle);
static void I2C_STOP_handler(I2C_Handle_t *pI2Chandle);
static void I2C_TC_handler(I2C_Handle_t *pI2Chandle);
static void I2C_TCR_handler(I2C_Handle_t *pI2Chandle); //when NBYTES>255

static void I2C_SlaveAddress(I2C_Handle_t *pI2Chandle); //set slave address

/*Enable or disable I2C peripheral clock*/
void I2C_ClockControl(I2C_Handle_t *pI2Chandle, uint8_t EnableDisable){
	if (EnableDisable == ENABLE) {
		if (pI2Chandle->pI2C == I2C1){
			I2C1_EnableClock();
		}
		else if (pI2Chandle->pI2C == I2C2){
			I2C2_EnableClock();
		}
	}
	else { //DISABLE
		if (pI2Chandle->pI2C == I2C1){
			I2C1_DisableClock();
		}
		else if (pI2Chandle->pI2C == I2C2){
			I2C2_DisableClock();
		}
	}
}

/*Initialization, which enables clock but not the peripheral yet(this is done by I2C_EnableDisable)*/
void I2C_Init(I2C_Handle_t *pI2Chandle){
	//enable I2C peripheral clock
	I2C_ClockControl(pI2Chandle,ENABLE);

	/*The timings must be configured in order to guarantee correct data hold and setup times,
	*used in master and slave modes. This is done by programming the PRESC[3:0],
	*SCLDEL[3:0] and SDADEL[3:0] bits in the I2C_TIMINGR register.
	*/
	/*Configure TIMINGR. Use I2C_Timing_Config_Tool_Vx.y.z.xls or check out values in 26.4.11
	 * I2C_TIMINGR register configuration examples.
	 * This includes clock speed, I2C mode (standard,fast...)
	 */
	pI2Chandle->pI2C->I2C_TIMINGR = (uint32_t)pI2Chandle->I2C_Config.I2C_Timing;

	/*//TX buffer as a dynamic array
	pI2Chandle->I2C_Comm_t.TX_buffer = malloc(pI2Chandle->I2C_Comm_t.TX_length);*/
}

/*I2C control: enable/disable */
void I2C_EnableDisable(I2C_Handle_t *pI2Chandle, uint8_t EnableDisable){
	if (EnableDisable == ENABLE){
		pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_PE);
		pI2Chandle->I2C_Comm_t.communication_state = I2C_READY;
	}
	else { //0
		pI2Chandle->pI2C->I2C_CR1 &= (~(1<<I2C_CR1_PE));
	}
}

/*Reset I2C */
void I2C_Reset(I2C_Handle_t *pI2Chandle){
	if (pI2Chandle->pI2C == I2C1){
		I2C1_ResetClock();
	}
	else if (pI2Chandle->pI2C == I2C2){
		I2C2_ResetClock();
	}
	/*PE must be kept low during at least three APB clock cycles in order to perform the software
	*reset.This is ensured by writing the following software sequence:1.Write PE = 0;
	*2.Check PE = 0;3.Write PE = 1*/
	pI2Chandle->pI2C->I2C_CR1 &= !((1<<I2C_CR1_PE));
	if (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_PE)){
		//do nothing, it's just step 2
	}
	pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_PE);
}

/*IRQ Configuration and ISR handling*/
void I2C_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable){ //IRQ_I2C1(combined with eXTI line23)/IRQ_I2C2
	if(EnableDisable == ENABLE){ //enable by means of ISER
		*NVIC_ISER |= ( 1 << IRQ_Number);
	}
	else { //disable by means of ICER
		*NVIC_ICER |= ( 1 << IRQ_Number);
	}
}

void I2C_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority){
	//if required, reuse the same function in spi (better a common function in stm32f091rct6.h?)
}

void I2C_IRQ_Handling(I2C_Handle_t *pI2Chandle){
	//Call the handler related to the interrupt if register is true as long as it is enabled
	//TXIS: Transmit Interrupt Status
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_TXIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_TXIS)) ){
		I2C_TXIS_handler(pI2Chandle);
	}
	//RXNE: Receive data register not empty
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_RXIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_RXNE)) ){
		I2C_RXNE_handler(pI2Chandle);
	}
	//BERR: bus error
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_ERRIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_BERR)) ){
		I2C_BERR_handler(pI2Chandle);
	}
	//TC: Transfer complete interrupt
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_TCIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_TC)) ){
		I2C_TC_handler(pI2Chandle);
	}
	//TCR: Transfer complete reload (enabled by TCIE too)
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_TCIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_TCR)) ){
		I2C_TCR_handler(pI2Chandle);
	}
	//STOP: Stop detection Interrupt
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_STOPIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_STOPF)) ){
		I2C_STOP_handler(pI2Chandle);
	}
}

static void I2C_SlaveAddress(I2C_Handle_t *pI2Chandle){
	//slave address (7-bit or 10-bit)
	if (pI2Chandle->I2C_Config.I2C_AddressMode == I2C_ADDRESS_MODE_7BIT){ //7-bit
		//mask to 7 bits and set
		pI2Chandle->pI2C->I2C_CR2 |= (pI2Chandle->I2C_Comm_t.I2C_SlaveAddress & 0x7F);
	}
	else { //10-bit
		//mask to 10 bits and set
		pI2Chandle->pI2C->I2C_CR2 |= (pI2Chandle->I2C_Comm_t.I2C_SlaveAddress & 0x3FF);
	}
}

void I2C_Master_Transmitter(I2C_Handle_t *pI2Chandle, uint8_t exp_bytes, uint8_t autoend){
	//TX state busy + configure transmission(including interrupts) + start&address
	if (pI2Chandle->I2C_Comm_t.communication_state != (I2C_DURING_TX || I2C_DURING_RX)){
		//start transmission (start+slave address) if I2C_READY,I2C_RESTART_STOP or I2C_RELOAD
		//slave address (7-bit or 10-bit)
		I2C_SlaveAddress(pI2Chandle);
		//enable TXIE,necessary for TXIS
		pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TXIE);
		//write NBYTES --> this would clear TCR by the way
		pI2Chandle->I2C_Comm_t.I2C_Nbytes = bytes;
		//if exp_bytes > 255 --> RELOAD = 1, enable TCR
		if (pI2Chandle->I2C_Comm_t.I2C_Nbytes > 255){ //RELOAD mode (AUTOEND does not apply)
			pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_RELOAD);
			pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //TCIE is for both TC and TCR
		}
		else{
			pI2Chandle->pI2C->I2C_CR2 &= !((1<<I2C_CR2_RELOAD)); //disable RELOAD
			/*AUTOEND
			 *0: software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
			 *1: Automatic end mode: a STOP condition is automatically sent when NBYTES data are
			 *transferred.
			 *Note: This bit has no effect in slave mode or when the RELOAD bit is set.
			 */
			if (!autoend){ //software end mode
				pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_AUTOEND);
				pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //to enable TC (TCIE is for both TC and TCR)
			}
			else{ //automatic end mode
				pI2Chandle->pI2C->I2C_CR2 &= !((1<<I2C_CR2_AUTOEND));
				pI2Chandle->pI2C->I2C_CR1 &= !((1<<I2C_CR1_TCIE)); //to disable TC
			}

		}
		/*if we needed something extra depending on previous state,enable this if-elseif
		if (pI2Chandle->I2C_Comm_t.communication_state == I2C_RELOAD){
			//do something extra?
		}
		else if (pI2Chandle->I2C_Comm_t.communication_state == (I2C_READY || I2C_RESTART_STOP)){
			//do something extra?
		}
		*/
		pI2Chandle->I2C_Comm_t.communication_state = I2C_DURING_TX;
		//start + address. this would clear TC if it had been set (RESTART_STOP)
		//This bit is set by SW and cleared by HW after Start followed by address sequence is sent
		pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_START);
		I2C_App_Callback(pI2Chandle, I2C_TX_STARTED);
	}

}

void I2C_Master_Receiver(I2C_Handle_t *pI2Chandle, uint8_t exp_bytes, uint8_t autoend){
	//RX state busy + configure reception(including interrupts) + start&address
	if (pI2Chandle->I2C_Comm_t.communication_state != (I2C_DURING_TX || I2C_DURING_RX)){
		//start transmission (start+slave address) if I2C_READY,I2C_RESTART_STOP or I2C_RELOAD
		//slave address (7-bit or 10-bit)
		I2C_SlaveAddress(pI2Chandle);
		//enable RXIE,necessary for RXNE
		pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_RXIE);
		//write NBYTES --> this would clear TCR by the way
		pI2Chandle->I2C_Comm_t.I2C_Nbytes = bytes;
		//if exp_bytes > 255 --> RELOAD = 1, enable TCR
		if (pI2Chandle->I2C_Comm_t.I2C_Nbytes > 255){ //RELOAD mode (AUTOEND does not apply)
			pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_RELOAD);
			pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //TCIE is for both TC and TCR
		}
		else{
			pI2Chandle->pI2C->I2C_CR2 &= !((1<<I2C_CR2_RELOAD)); //disable RELOAD
			/*AUTOEND
			 *0: software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
			 *1: Automatic end mode: a STOP condition is automatically sent when NBYTES data are
			 *transferred.
			 *Note: This bit has no effect in slave mode or when the RELOAD bit is set.
			 */
			if (!autoend){ //software end mode
				pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_AUTOEND);
				pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //to enable TC (TCIE is for both TC and TCR)
			}
			else{ //automatic end mode
				pI2Chandle->pI2C->I2C_CR2 &= !((1<<I2C_CR2_AUTOEND));
				pI2Chandle->pI2C->I2C_CR1 &= !((1<<I2C_CR1_TCIE)); //to disable TC
			}

		}
		/*if we needed something extra depending on previous state,enable this if-elseif
		if (pI2Chandle->I2C_Comm_t.communication_state == I2C_RELOAD){
			//something extra?
		}
		else if (pI2Chandle->I2C_Comm_t.communication_state == (I2C_READY || I2C_RESTART_STOP)){
			//something extra?
		}
		*/
		pI2Chandle->I2C_Comm_t.communication_state = I2C_DURING_RX;
		//start + address. this would clear TC if it had been set (RESTART_STOP)
		//This bit is set by SW and cleared by HW after Start followed by address sequence is sent
		pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_START);
		I2C_App_Callback(pI2Chandle, I2C_RX_STARTED);
	}
}
//to be done: same but in case of STM32 being slave

//to send STOP when autoend=0.called in main program
void I2C_Master_Manual_Stop(I2C_Handle_t *pI2Chandle){
	if (pI2Chandle->I2C_Comm_t.communication_state == I2C_RESTART_STOP){
		//bit set by SW, cleared by HW when a STOP condition is detected.Writing ‘0’ has no effect
		pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_STOP);
	}
}

static void I2C_TXIS_handler(I2C_Handle_t *pI2Chandle){
	/*TXIS set by hardware when I2C_TXDR register empty and the data to be
	*transmitted must be written in I2C_TXDR. It is cleared when the next data to be
	*sent is written in I2C_TXDR register*/
	if (pI2Chandle->I2C_Comm_t.TX_length >=1){
		pI2Chandle->pI2C->I2C_TXDR = *(pI2Chandle->I2C_Comm_t.TX_buffer);
		pI2Chandle->I2C_Comm_t.TX_buffer++;
		pI2Chandle->I2C_Comm_t.TX_length--;
		/*if last byte to be send,wait until STOP or TC/TCR,depending on AUTOEND(these interrupts are handled)
		*state=WAITING_END (STOP or TC/TCR flag)
		*if NBYTES>255, I2C_RELOAD*/
		if (pI2Chandle->I2C_Comm_t.TX_length == 0){
			pI2Chandle->I2C_Comm_t.communication_state = I2C_WAITING_END;
		}
	}

}

static void I2C_RXNE_handler(I2C_Handle_t *pI2Chandle){
	/*RXNE is set by hardware when received data is copied into I2C_RXDR register, and
	*is ready to be read. It is cleared when I2C_RXDR is read*/
	if (pI2Chandle->I2C_Comm_t.RX_length >=1){
		*(pI2Chandle->I2C_Comm_t.RX_buffer) = pI2Chandle->pI2C->I2C_RXDR;
		pI2Chandle->I2C_Comm_t.RX_buffer++;
		pI2Chandle->I2C_Comm_t.RX_length--;
		/*if last byte to be send,wait until STOP or TC/TCR,depending on AUTOEND(these interrupts are handled)
		*state=WAITING_END (STOP or TC/TCR flag)
		*if NBYTES>255, I2C_RELOAD*/
		if (pI2Chandle->I2C_Comm_t.RX_length == 0){
			pI2Chandle->I2C_Comm_t.communication_state = I2C_WAITING_END;
		}
	}

	//inform the main application of a new read value, through callback
	I2C_App_Callback(pI2Chandle, I2C_NEW_READING);
}

static void I2C_BERR_handler(I2C_Handle_t *pI2Chandle){
	/*BERR is set by hardware when a misplaced Start or STOP is detected whereas
	*the peripheral is involved in the transfer. The flag is not set during address phase in slave
	*mode. It is cleared by software by setting BERRCF bit*/
	pI2Chandle->pI2C->I2C_ICR |= (1<<I2C_ICR_BERRCF);
	//Disable I2C
	I2C_EnableDisable(pI2Chandle,DISABLE);
	//inform the main application through callback
	I2C_App_Callback(pI2Chandle, I2C_BERR_ERROR);
}

static void I2C_STOP_handler(I2C_Handle_t *pI2Chandle){
	/*This flag is set by hardware when a STOP condition is detected on the bus and the
	*peripheral is involved in this transfer. It is cleared by software by setting the STOPCF bit.*/
	//if I2C_WAITING_END --> I2C_READY. Close transmission/reception,buffer pointing to NULL
	if (pI2Chandle->I2C_Comm_t.communication_state == I2C_WAITING_END){
		pI2Chandle->I2C_Comm_t.communication_state = I2C_READY;
		pSPIhandle->SPI_Comm.RX_buffer = NULL; //null pointer
		pSPIhandle->SPI_Comm.TX_buffer = NULL; //null pointer
		pI2Chandle->pI2C->I2C_ICR |= (1<<I2C_ICR_STOPCF);
		I2C_EnableDisable(pI2Chandle,DISABLE); //Disable I2C
		I2C_App_Callback(pI2Chandle, I2C_FINISHED);
	}

}

static void I2C_TC_handler(I2C_Handle_t *pI2Chandle){
	/*TC(transfer complete) is set by hardware when RELOAD=0, AUTOEND=0 and NBYTES data have been
	*transferred. It is cleared by software when START bit or STOP bit is set.*/
	//if I2C_WAITING_END --> I2C_RESTART_STOP
	if (pI2Chandle->I2C_Comm_t.communication_state == I2C_WAITING_END){
		pI2Chandle->I2C_Comm_t.communication_state = I2C_RESTART_STOP;
		I2C_App_Callback(pI2Chandle, I2C_TC);
	}
}

static void I2C_TCR_handler(I2C_Handle_t *pI2Chandle){ //when NBYTES>255
	/*TCR(transfer complete reload) is set by hardware when RELOAD=1 and NBYTES data have been transferred. It is
	*cleared by software when NBYTES is written to a non-zero value.*/
	//if I2C_WAITING_END --> I2C_RELOAD
	if (pI2Chandle->I2C_Comm_t.communication_state == I2C_WAITING_END){
		pI2Chandle->I2C_Comm_t.communication_state = I2C_RELOAD;
		I2C_App_Callback(pI2Chandle, I2C_TCR);
	}
}

/*Callback to application */
__attribute__((weak)) I2C_App_Callback(I2C_Handle_t *pI2Chandle,uint8_t Event){
	//This function will be implemented in every particular application. Thus, the weak attribute
}
