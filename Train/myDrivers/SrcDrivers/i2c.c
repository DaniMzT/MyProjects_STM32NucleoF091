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
	 * In case of knowing PRESC,SCLDEL,SDADEL,SCLH and SCLL: use my I2C_Master_TimingR
	 */
	pI2Chandle->pI2C->I2C_TIMINGR = pI2Chandle->I2C_Config.I2C_Timing;

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
		pI2Chandle->pI2C->I2C_CR1 &= ~(1<<I2C_CR1_PE);
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
	pI2Chandle->pI2C->I2C_CR1 &= ~(1<<I2C_CR1_PE);
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
	//7 IPR registers, each one in 8-bits blocks for every IRQ. So 4 IRQ per IPR
	/*Each priority field holds a priority value, 0-192. The lower the value, the greater the priority of the corresponding interrupt.
	 * The processor implements only bits[7:6] of each field, bits [5:0] read as zero and ignore writes
	 * Programmable priority level 0-192 in steps of 64 for each interrupt. level 0 is the highest interrupt priority.*/
	if (IRQ_Priority>255) { //in order not to affect the next blocks
		IRQ_Priority=255;
	}
	uint8_t numberIPR = IRQ_Number / 4;
	uint8_t blockIPR = IRQ_Number % 4;
	//Clear the block and set. Better if I use intermediate variables instead of modifying many times the IPR
	uint32_t tempIPR = ((*(NVIC_IPR_BASE + numberIPR))&~(0xFF<<blockIPR));
	tempIPR |= (IRQ_Priority<<(8*blockIPR+6)); //6 is because bits 0-5 are not editable, only bits 6-7 count.
	*(NVIC_IPR_BASE + numberIPR) = tempIPR;
}

void I2C_IRQ_Handling(I2C_Handle_t *pI2Chandle){
	//Call the handler related to the interrupt if register is true as long as it is enabled
	//STOP: Stop detection Interrupt.Place it first to disable I2C soon (especially not to overrun RX).
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_STOPIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_STOPF)) ){
		I2C_STOP_handler(pI2Chandle);
		return;
	}
	//TXIS: Transmit Interrupt Status
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_TXIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_TXIS)) ){
		I2C_TXIS_handler(pI2Chandle);
		return;
	}
	//RXNE: Receive data register not empty
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_RXIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_RXNE)) ){
		I2C_RXNE_handler(pI2Chandle);
		return;
	}
	//BERR: bus error
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_ERRIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_BERR)) ){
		I2C_BERR_handler(pI2Chandle);
		return;
	}
	//TC: Transfer complete interrupt
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_TCIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_TC)) ){
		I2C_TC_handler(pI2Chandle);
		return;
	}
	//TCR: Transfer complete reload (enabled by TCIE too)
	if ( (pI2Chandle->pI2C->I2C_CR1 & (1<<I2C_CR1_TCIE)) && (pI2Chandle->pI2C->I2C_ISR & (1<<I2C_ISR_TCR)) ){
		I2C_TCR_handler(pI2Chandle);
		return;
	}

}

static void I2C_SlaveAddress(I2C_Handle_t *pI2Chandle){
	//slave address (7-bit or 10-bit)
	if (pI2Chandle->I2C_Config.I2C_AddressMode == I2C_ADDRESS_MODE_7BIT){ //7-bit
		//mask to 7 bits, move 1 bit to left (7-bits, so from b1 to b7) and set
		pI2Chandle->pI2C->I2C_CR2 |= ((pI2Chandle->I2C_Comm_t.I2C_SlaveAddress & 0x7F)<<1);
	}
	else { //10-bit
		//mask to 10 bits and set
		pI2Chandle->pI2C->I2C_CR2 |= (pI2Chandle->I2C_Comm_t.I2C_SlaveAddress & 0x3FF);
	}
}

void I2C_Start(I2C_Handle_t *pI2Chandle){
	pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_START);
}

void I2C_Master_Transmitter(I2C_Handle_t *pI2Chandle, uint8_t exp_bytes, uint8_t autoend, uint8_t* appTxBuffer){
	//TX state busy + configure transmission(including interrupts) + start&address
	if (pI2Chandle->I2C_Comm_t.communication_state != (I2C_DURING_TX || I2C_DURING_RX)){
		//start transmission (start+slave address) if I2C_READY,I2C_RESTART_STOP or I2C_RELOAD
		//transfer direction: master requests a write transfer (0)
		pI2Chandle->pI2C->I2C_CR2 &= ~(1<<I2C_CR2_RDWRN);
		//slave address (7-bit or 10-bit)
		I2C_SlaveAddress(pI2Chandle);
		//enable TXIE,necessary for TXIS
		pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TXIE);
		pI2Chandle->I2C_Comm_t.I2C_Nbytes = exp_bytes;
		//if exp_bytes > 255 --> RELOAD = 1, enable TCR
		if (pI2Chandle->I2C_Comm_t.I2C_Nbytes > 255){ //RELOAD mode (AUTOEND does not apply)
			pI2Chandle->pI2C->I2C_CR2 |= (255<<I2C_CR2_NBYTES); //write NBYTES --> this would clear TCR by the way
			pI2Chandle->I2C_Comm_t.TX_length = 255;
			pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_RELOAD);
			pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //TCIE is for both TC and TCR
		}
		else{
			//writing NBYTES clears TCR by the way.first reset NBYTES (important,8bits!) and then set them
			pI2Chandle->pI2C->I2C_CR2 &= ~(255<<I2C_CR2_NBYTES);//reset NBYTES. 0xFF=1..1
			pI2Chandle->pI2C->I2C_CR2 |= (exp_bytes<<I2C_CR2_NBYTES); //set NBYTES
			pI2Chandle->I2C_Comm_t.TX_length = exp_bytes;
			pI2Chandle->pI2C->I2C_CR2 &= ~(1<<I2C_CR2_RELOAD); //disable RELOAD
			/*AUTOEND
			 *0: software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
			 *1: Automatic end mode: a STOP condition is automatically sent when NBYTES data are
			 *transferred.
			 *Note: This bit has no effect in slave mode or when the RELOAD bit is set.
			 */
			if (autoend){ //automatic end mode
				pI2Chandle->I2C_Comm_t.I2C_RepeatStart = I2C_AUTOEND;
				pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_AUTOEND);
				pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //to enable TC (TCIE is for both TC and TCR)
				pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_STOPIE); //to enable STOP detection
			}
			else{ //software end mode
				pI2Chandle->I2C_Comm_t.I2C_RepeatStart = I2C_REPEAT_START;
				pI2Chandle->pI2C->I2C_CR2 &= ~(1<<I2C_CR2_AUTOEND);
				pI2Chandle->pI2C->I2C_CR1 &= ~(1<<I2C_CR1_TCIE); //to disable TC
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
		pI2Chandle->I2C_Comm_t.TX_buffer = appTxBuffer; //I2C TX buffer pointing to app's TX buffer
		pI2Chandle->I2C_Comm_t.communication_state = I2C_DURING_TX;
		//start + address. this would clear TC if it had been set (RESTART_STOP)
		//This bit is set by SW and cleared by HW after Start followed by address sequence is sent
		//in case of problems, trigger START from the app
		pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_START);
	}

}

void I2C_Master_Receiver(I2C_Handle_t *pI2Chandle, uint8_t exp_bytes, uint8_t autoend, uint8_t* appRxBuffer){
	//RX state busy + configure reception(including interrupts) + start&address
	if (pI2Chandle->I2C_Comm_t.communication_state != (I2C_DURING_TX || I2C_DURING_RX)){
		//start transmission (start+slave address) if I2C_READY,I2C_RESTART_STOP or I2C_RELOAD
		//transfer direction: master requests a read transfer (1)
		pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_RDWRN);
		//slave address (7-bit or 10-bit)
		I2C_SlaveAddress(pI2Chandle);
		//enable RXIE,necessary for RXNE
		pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_RXIE);
		pI2Chandle->I2C_Comm_t.I2C_Nbytes = exp_bytes;
		//if exp_bytes > 255 --> RELOAD = 1, enable TCR
		if (pI2Chandle->I2C_Comm_t.I2C_Nbytes > 255){ //RELOAD mode (AUTOEND does not apply)
			pI2Chandle->pI2C->I2C_CR2 |= (255<<I2C_CR2_NBYTES); //write NBYTES --> this would clear TCR by the way
			pI2Chandle->I2C_Comm_t.RX_length = 255;
			pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_RELOAD);
			pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //TCIE is for both TC and TCR
		}
		else{
			//writing NBYTES clears TCR by the way.first reset NBYTES (important,8bits!) and then set them
			pI2Chandle->pI2C->I2C_CR2 &= ~(255<<I2C_CR2_NBYTES);//reset NBYTES. 0xFF=1..1
			pI2Chandle->pI2C->I2C_CR2 |= (exp_bytes<<I2C_CR2_NBYTES); //set NBYTES
			pI2Chandle->I2C_Comm_t.RX_length = exp_bytes;
			pI2Chandle->pI2C->I2C_CR2 &= ~(1<<I2C_CR2_RELOAD); //disable RELOAD
			/*AUTOEND
			 *0: software end mode: TC flag is set when NBYTES data are transferred, stretching SCL low.
			 *1: Automatic end mode: a STOP condition is automatically sent when NBYTES data are
			 *transferred.
			 *Note: This bit has no effect in slave mode or when the RELOAD bit is set.
			 */
			if (autoend){ //automatic end mode
				pI2Chandle->I2C_Comm_t.I2C_RepeatStart = I2C_AUTOEND;
				pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_AUTOEND);
				pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_TCIE); //to enable TC (TCIE is for both TC and TCR)
				pI2Chandle->pI2C->I2C_CR1 |= (1<<I2C_CR1_STOPIE); //to enable STOP detection
			}
			else{ //software end mode
				pI2Chandle->I2C_Comm_t.I2C_RepeatStart = I2C_REPEAT_START;
				pI2Chandle->pI2C->I2C_CR2 &= ~(1<<I2C_CR2_AUTOEND);
				pI2Chandle->pI2C->I2C_CR1 &= ~(1<<I2C_CR1_TCIE); //to disable TC
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
		pI2Chandle->I2C_Comm_t.RX_buffer = appRxBuffer; //I2C RX buffer pointing to app's RX buffer
		pI2Chandle->I2C_Comm_t.communication_state = I2C_DURING_RX;
		//start + address. this would clear TC if it had been set (RESTART_STOP)
		//This bit is set by SW and cleared by HW after Start followed by address sequence is sent
		//in case of problems (RXNE delays for example), trigger START from the app
		pI2Chandle->pI2C->I2C_CR2 |= (1<<I2C_CR2_START);
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
	if (pI2Chandle->I2C_Comm_t.RX_length >= 1){
		*pI2Chandle->I2C_Comm_t.RX_buffer = ((uint8_t)pI2Chandle->pI2C->I2C_RXDR); //read RXDR
		pI2Chandle->I2C_Comm_t.RX_buffer++; //increase address which I2C buffer points to (next position app's RXbuffer);next RXDR is stored there
		pI2Chandle->I2C_Comm_t.RX_length--;
		/*if last byte to be send,wait until STOP or TC/TCR,depending on AUTOEND(these interrupts are handled)
		*state=WAITING_END (STOP or TC/TCR flag)
		*if NBYTES>255, I2C_RELOAD*/
		/*if ((pI2Chandle->I2C_Comm_t.I2C_RepeatStart == I2C_AUTOEND) && (pI2Chandle->I2C_Comm_t.RX_length <= 1 )){
			pI2Chandle->I2C_Comm_t.communication_state = I2C_WAITING_END; //autoend: STOP
			//no callback, STOP handler will do
		}*/
		if (pI2Chandle->I2C_Comm_t.RX_length == 0){
			pI2Chandle->I2C_Comm_t.communication_state = I2C_WAITING_END;
			I2C_App_Callback(pI2Chandle, I2C_NEW_READING);
		}
	}

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
	/*if (pI2Chandle->I2C_Comm_t.communication_state == I2C_WAITING_END){
		pI2Chandle->I2C_Comm_t.communication_state = I2C_READY;
		pI2Chandle->pI2C->I2C_ICR |= (1<<I2C_ICR_STOPCF);
		I2C_App_Callback(pI2Chandle, I2C_FINISHED);
		//Disable I2C in main application
	}*/
	pI2Chandle->I2C_Comm_t.communication_state = I2C_READY;
	pI2Chandle->pI2C->I2C_ICR |= (1<<I2C_ICR_STOPCF);
	I2C_App_Callback(pI2Chandle, I2C_FINISHED);
	//Disable I2C in main application
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
__attribute__((weak)) void I2C_App_Callback(I2C_Handle_t *pI2Chandle,uint8_t Event){
	//This function will be implemented in every particular application. Thus, the weak attribute
}

/*TIMINGR calculation*/
void I2C_Master_TimingR(I2C_Handle_t *pI2Chandle, uint8_t presc, uint8_t scldel, uint8_t sdadel, uint8_t sclh, uint8_t scll){
	pI2Chandle->I2C_Config.I2C_Timing |= ((presc & 0x0F) << 28);
	pI2Chandle->I2C_Config.I2C_Timing |= ((scldel & 0x0F) << 20);
	pI2Chandle->I2C_Config.I2C_Timing |= ((sdadel & 0x0F) << 16);
	pI2Chandle->I2C_Config.I2C_Timing |= (sclh << 8);
	pI2Chandle->I2C_Config.I2C_Timing |= scll;
}
