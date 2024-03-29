/*
 * gpio.c
 *
 *  Created on: May 19, 2022
 *      Author: danim
 */

#include "gpio.h"
/*********GPIO clock control***************/
void GPIO_ClockControl(GPIO_RegStruct_t *pGPIO, uint8_t EnableDisable){

	if (EnableDisable == ENABLE) {
		if (pGPIO==GPIOA) { //I can't do switch-case with pointers! that's why I use if
			GPIOA_EnableClock();
		}
		else if (pGPIO==GPIOB){
			GPIOB_EnableClock();
		}
		else if (pGPIO==GPIOC){
			GPIOC_EnableClock();
		}
		else if (pGPIO==GPIOD){
			GPIOD_EnableClock();
		}
		else if (pGPIO==GPIOE){
			GPIOE_EnableClock();
		}
		else if (pGPIO==GPIOF){
			GPIOF_EnableClock();
		}
	}
	else{
		if (pGPIO==GPIOA) { //I can't do switch-case with pointers! that's why I use if
			GPIOA_DisableClock();
		}
		else if (pGPIO==GPIOB){
			GPIOB_DisableClock();
		}
		else if (pGPIO==GPIOC){
			GPIOC_DisableClock();
		}
		else if (pGPIO==GPIOD){
			GPIOD_DisableClock();
		}
		else if (pGPIO==GPIOE){
			GPIOE_DisableClock();
		}
		else if (pGPIO==GPIOF){
			GPIOF_DisableClock();
		}
	}
}
/*********GPIO pin initialization*************/
void GPIO_PinInit(GPIO_PinHandle_t *GPIO_PinHandle){

	//local variables used for temporary values in operations
	uint8_t temp1;
	uint8_t temp2;

	//enable GPIO port clock
	GPIO_ClockControl(GPIO_PinHandle->pGPIO,ENABLE);

	//configure MODE: if not interrupt, mode is input/output/analog/alternate function
	if ((GPIO_PinHandle->GPIO_PinConfig.GPIO_PinMode)<=GPIO_ANA){
		// first,reset the 2 bits related to pin.ex:pin number 1 is for MODER bits 2 and 3. 3<<2*pin is 3<<2 aka 1100b.so MODER&=(~1100)=x..x00xx
		GPIO_PinHandle->pGPIO->MODER &= (~(3<<(2*GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber))); //I guess I can use 3 instead of 0x3
		//now set
		GPIO_PinHandle->pGPIO->MODER |= (GPIO_PinHandle->GPIO_PinConfig.GPIO_PinMode<<(2*GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	else{ //interrupts
		//Configure the edge trigger (EXTI FTSR and RTSR). EXTI line "y" is for pins "y"; example, EXTI15 is for GPIOA15..GPIOF15
		if ((GPIO_PinHandle->GPIO_PinConfig.GPIO_PinMode)==GPIO_FALL_TRIG){
			//set falling edge trigger
			EXTI->FTSR |= (1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear rising edge trigger!
			EXTI->RTSR &= ~(1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if ((GPIO_PinHandle->GPIO_PinConfig.GPIO_PinMode)==GPIO_RISE_TRIG){
			//set rising edge trigger
			EXTI->RTSR |= (1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear falling edge trigger!
			EXTI->FTSR &= ~(1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if ((GPIO_PinHandle->GPIO_PinConfig.GPIO_PinMode)==GPIO_FALLRISE_TRIG){
			//set both falling and rising edge triggers
			EXTI->FTSR |= (1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//EXTICR: select the GPIO for the interruption on corresponding EXTI line through EXTICR registers (every allows just one, so it's a decision)
		//EXTICR0:for GPIO pins 0-3. Blocks of 4 bits to select A-F.EXTICR1: for GPIO pins 4-7. Blocks of 4 bits to select A-F. And so on
		//if GPIOD10-->EXTICR3 (block1,that is,from bit4) because EXTICR1 is for pins 0-3, CR2 pins 4-7 and so on. GPIOD--> 010
		temp1 = GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber/4; //to know which EXTICR
		temp2 = GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber%4;//to know the block within EXTICR
		//enable SYSCFG clock and configure the correspondingEXTICR
		SYSCFG_EnableClock();
		SYSCFG->EXTICR[temp1] |= ((GPIO_PORT_TO_NUMBER(GPIO_PinHandle->pGPIO))<< (4*temp2) );

		//ENABLE EXTI interrupt by means of interrupt mask register
		EXTI->IMR |= (1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber);

		/*Check specific functions for the configuration on processor side: NVIC (nested vector interrupt controller)*/
	}

	//OUTPUT TYPE. TBD:don't do if !=output mode?
	// first,reset the related bit
	GPIO_PinHandle->pGPIO->OTYPER &= (~(1<<GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber));
	//now set
	GPIO_PinHandle->pGPIO->OTYPER |= (GPIO_PinHandle->GPIO_PinConfig.GPIO_PinOutType<<(GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber));

	//OUTPUT SPEED. TBD:don't do if !=output mode?
	// first,reset the related bits
	GPIO_PinHandle->pGPIO->OSPEEDR &= (~(3<<(2*GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber)));
	//now set
	GPIO_PinHandle->pGPIO->OSPEEDR |= (GPIO_PinHandle->GPIO_PinConfig.GPIO_PinOutSpeed<<(2*GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber));

	//PULL UP/DOWN
	// first,reset the related bits
	GPIO_PinHandle->pGPIO->PUPDR &= (~(3<<(2*GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber)));
	//now set
	GPIO_PinHandle->pGPIO->PUPDR |= (GPIO_PinHandle->GPIO_PinConfig.GPIO_PinPullUpDown<<(2*GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber));

	//ALTERNATE FUNCTION
	if (GPIO_PinHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_ALTFUN){
		//as there are 2 registers, low is for pins 0-7 and high for 8-15.in our struct,AFR[2].We need to know where to actuate
		temp1 = GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ; //ex: pin 7-->reg 0,byte=7(last);pin12 --> reg1, byte=4
		temp2 = GPIO_PinHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		// first,reset the related bits. in this case, 4. ~1111 is ~0x0F
		GPIO_PinHandle->pGPIO->AFR[temp1] &= (~(0xF<<(4*temp2)));
		//now set
		GPIO_PinHandle->pGPIO->AFR[temp1] |= (GPIO_PinHandle->GPIO_PinConfig.GPIO_PinAlterFunc<<(4*temp2));
	}
}
/*********GPIO port reset**********************/
void GPIO_ResetPort(GPIO_RegStruct_t *pGPIO){
	if(pGPIO==GPIOA) {
		GPIOA_Reset();
	}
	else if (pGPIO==GPIOB){
		GPIOB_Reset();
	}
	else if (pGPIO==GPIOC){
		GPIOC_Reset();
	}
	else if (pGPIO==GPIOD){
		GPIOD_Reset();
	}
	else if (pGPIO==GPIOE){
		GPIOE_Reset();
	}
	else if (pGPIO==GPIOF){
		GPIOF_Reset();
	}
}

/***************************Read from input/port*******************************/
uint8_t GPIO_ReadPin(GPIO_RegStruct_t *pGPIO, uint8_t pin){
	//to read pin say 5,right shift 5 pos so that pin is bit0 and then clear rest of bits.(xx..110010 <<5)&=1 ->(00000xx..1)&=1 -> 00..01
	return (uint8_t)((pGPIO->IDR << pin)&(0x1));
}
uint16_t GPIO_ReadPort(GPIO_RegStruct_t *pGPIO){
	return (uint16_t)pGPIO->IDR;
}

/**************************Write to output (pin or port)************************/
void GPIO_WritePin(GPIO_RegStruct_t *pGPIO, uint8_t pin, uint8_t output){
	//do not do first clear the related bit and then set.we don't want to modify the output
	//I prefer to do an if statement than using intermediate variables to clear and set
	if (output==1){
		pGPIO->ODR |= (1<<pin);
	}
	else { //0
		pGPIO->ODR &= (~(1<<pin));
	}

}
void GPIO_WritePort(GPIO_RegStruct_t *pGPIO, uint16_t output){
	pGPIO->ODR = output;
}
void GPIO_TogglePin(GPIO_RegStruct_t *pGPIO, uint8_t pin){
	pGPIO->ODR ^= (1<<pin); //^ is XOR. 1^0=0;0^1=1;1^1=0;0^0=0. So x^1 = !x
}

/*************************NVIC functions for IRQ*****************************************/
/*Enable or disable an IRQ*/
void GPIO_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable){
	if(EnableDisable == ENABLE){ //enable by means of ISER
		*NVIC_ISER |= ( 1 << IRQ_Number);
	}
	else { //disable by means of ICER
		*NVIC_ICER |= ( 1 << IRQ_Number);
	}

}
void GPIO_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority){
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
void GPIO_IRQ_Handling(uint8_t pin){
	/*ISRs (handlers) are actually application specific and implement/override(weak) handlers defined in startup*/
	/*GPIO_IRQ_Handling will be called inside the ISR handler.*/
	//Clear the pending register of the EXTI(interrupt handling depends upon the peripheral).PR of processor is automatically cleared (I think)
	if ((EXTI->PR) & (1<<pin)){ //if PR[pin]==1
		(EXTI->PR) |= (1<<pin); //write 1 to clear the pending register
	}

}
