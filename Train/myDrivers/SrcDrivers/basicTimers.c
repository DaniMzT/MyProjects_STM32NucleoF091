/*
 * basicTimers.c
 *
 *  Created on: Dec 28, 2022
 *      Author: danimt
 */

#include "basicTimers.h"

/*Enable or disable BasicTimer peripheral clock*/
void BasicTimer_ClockControl(BasicTimer_Handle_t *pBasicTimerhandle, uint8_t EnableDisable){
	if (EnableDisable == ENABLE) {
		if (pBasicTimerhandle->pBasicTimer == BASIC_TIMER_6){
			BasicTimer_6_Clock_Enable();
		}
		else if (pBasicTimerhandle->pBasicTimer == BASIC_TIMER_7){
			BasicTimer_7_Clock_Enable();
		}
	}
	else {
		if (pBasicTimerhandle->pBasicTimer == BASIC_TIMER_6){
			BasicTimer_6_Clock_Disable();
		}
		else if (pBasicTimerhandle->pBasicTimer == BASIC_TIMER_7){
			BasicTimer_7_Clock_Disable();
		}
	}
}

/*BasicTimer initialization*/
void BasicTimer_Init(BasicTimer_Handle_t *pBasicTimerhandle){
	//enable the clock
	BasicTimer_ClockControl(pBasicTimerhandle,ENABLE);
    //first,CR1 can be reset completely as it is not related to other parts (it might be reset with memset in the app,too)
	pBasicTimerhandle->pBasicTimer->TIM_CR1 = 0; //this allows avoiding if(enable)-else and instead do |=
	//ARR is 0xFFFF is reset value
    pBasicTimerhandle->pBasicTimer->TIM_CR1 |= (pBasicTimerhandle->BasicTimer_Config.BasicTimer_arpe << TIM_CR1_ARPE);
    pBasicTimerhandle->pBasicTimer->TIM_CR1 |= (pBasicTimerhandle->BasicTimer_Config.BasicTimer_opm << TIM_CR1_OPM);
    pBasicTimerhandle->pBasicTimer->TIM_CR1 |= (pBasicTimerhandle->BasicTimer_Config.BasicTimer_urs << TIM_CR1_URS);
    pBasicTimerhandle->pBasicTimer->TIM_CR1 |= (pBasicTimerhandle->BasicTimer_Config.BasicTimer_udis << TIM_CR1_UDIS);
    pBasicTimerhandle->pBasicTimer->TIM_PSC = pBasicTimerhandle->BasicTimer_Config.BasicTimer_prescaler;
    pBasicTimerhandle->pBasicTimer->TIM_ARR = pBasicTimerhandle->BasicTimer_Config.BasicTimer_autoreload;
}

/*BasicTimer control: enable/disable --> CEN */
void BasicTimer_EnableDisable(BasicTimer_Handle_t *pBasicTimerhandle, uint8_t EnableDisable){
    if (EnableDisable == ENABLE){
        pBasicTimerhandle->pBasicTimer->TIM_CR1 |= (1<<TIM_CR1_CEN); //enable counter
        pBasicTimerhandle->pBasicTimer->TIM_DIER |= (1<<TIM_DIER_UIE); //enable interrupts
    }
    else{
        pBasicTimerhandle->pBasicTimer->TIM_CR1 &= ~(1<<TIM_CR1_CEN);
        pBasicTimerhandle->pBasicTimer->TIM_DIER &= ~(1<<TIM_DIER_UIE);
    }
}

/*Reset BasicTimer */
void BasicTimer_Reset(BasicTimer_Handle_t *pBasicTimerhandle){

    if (pBasicTimerhandle->pBasicTimer == BASIC_TIMER_6){
        BasicTimer_6_Clock_Reset();
    }
    else if (pBasicTimerhandle->pBasicTimer == BASIC_TIMER_7){
        BasicTimer_7_Clock_Reset();
    }
}

/*Get the counter value*/
uint16_t BasicTimer_GetCounter(BasicTimer_Handle_t *pBasicTimerhandle){
    return pBasicTimerhandle->pBasicTimer->TIM_CNT;
}

/*Restart counter*/
void BasicTimer_RestartCounter(BasicTimer_Handle_t *pBasicTimerhandle){
    pBasicTimerhandle->pBasicTimer->TIM_EGR |= (1<<TIM_EGR_UG);
}

/*IRQ Configuration and ISR handling*/
void BasicTimer_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable){
	if(EnableDisable == ENABLE){ //enable by means of ISER
		*NVIC_ISER |= ( 1 << IRQ_Number);
	}
	else { //disable by means of ICER
		*NVIC_ICER |= ( 1 << IRQ_Number);
	}
}
void BasicTimer_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority){
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
void BasicTimer_IRQ_Handling(BasicTimer_Handle_t *pBasicTimerhandle){
    if ((pBasicTimerhandle->pBasicTimer->TIM_DIER |= (1<<TIM_DIER_UIE)) && (pBasicTimerhandle->pBasicTimer->TIM_SR |= (1<<TIM_SR_UIF))){
        //NOT NEEDED AS THERE IS ONLY ONE INTERRUPT BIT Not BasicTimer_UpdateInterruptFlag_handler(pBasicTimerhandle);
        BasicTimer_App_Callback(pBasicTimerhandle); //callback to execute operations related to timer interrupt
        pBasicTimerhandle->pBasicTimer->TIM_SR &= ~(1<<TIM_SR_UIF); //clear the update interrupt flag
    }
}

/*Static functions, interrupt handlers*/
/*NOT NEEDED AS THERE IS ONLY ONE INTERRUPT BIT
static void BasicTimer_UpdateInterruptFlag_handler(BasicTimer_Handle_t *pBasicTimerhandle);
*/

/*Callback to application */
__attribute__((weak)) void BasicTimer_App_Callback(BasicTimer_Handle_t *pBasicTimerhandle){
    //This function will be implemented in every particular application. Thus, the weak attribute
}

/*Update disable UDIS=update disable-> 0:UEV enabled; 1:UEV disabled */
void BasicTimer_UDIS(BasicTimer_Handle_t *pBasicTimerhandle, uint8_t EnableDisable){
    if (EnableDisable == ENABLE){
        pBasicTimerhandle->pBasicTimer->TIM_CR1 |= (1 << TIM_CR1_UDIS);
    }
    else{
        pBasicTimerhandle->pBasicTimer->TIM_CR1 &= ~(1<<TIM_CR1_UDIS);
    }
}

/*Virtual basic timer functions*/
enum VirtualBasicTimer_IntervalStatus VirtualBasicTimer_Elapsed(VirtualBasicTimer_t *pVirtualBasicTimerhandle){
    if (pVirtualBasicTimerhandle->status == VIRTUAL_BASIC_TIMER_ACTIVE){
		pVirtualBasicTimerhandle->elapsed_counter_value = BasicTimer_GetCounter(&pVirtualBasicTimerhandle->realTimer)-pVirtualBasicTimerhandle->start_counter_value;
		//if the counter is lower than start_counter_value, that's because an overflow has happened! take this into account
		if (pVirtualBasicTimerhandle->elapsed_counter_value < 0){
			pVirtualBasicTimerhandle->elapsed_counter_value = pVirtualBasicTimerhandle->realTimer.BasicTimer_Config.BasicTimer_autoreload + pVirtualBasicTimerhandle->elapsed_counter_value;
		}
		//take into account the timer value before stopped
		pVirtualBasicTimerhandle->elapsed_counter_value += pVirtualBasicTimerhandle->counter_value_preStop;
		if (pVirtualBasicTimerhandle->elapsed_counter_value >= pVirtualBasicTimerhandle->configured_counter_value){
			//VirtualBasicTimer_App_Callback(pVirtualBasicTimerhandle); //let the main app know
			//VirtualBasicTimer_StartCounter(pVirtualBasicTimerhandle); //restart the counter
			return VIRTUAL_BASIC_TIMER_ELAPSED;
		}
		else{
			return VIRTUAL_BASIC_TIMER_NOT_ELAPSED;
		}
    }
    //else
    return FUNCTION_NOT_ACCESSIBLE;

}
void VirtualBasicTimer_EnableDisable(VirtualBasicTimer_t *pVirtualBasicTimerhandle, uint8_t EnableDisable){
    if (EnableDisable == ENABLE){
       pVirtualBasicTimerhandle->status = VIRTUAL_BASIC_TIMER_ACTIVE;
    }
    else{
        pVirtualBasicTimerhandle->status = VIRTUAL_BASIC_TIMER_INACTIVE;
    }
}
void VirtualBasicTimer_StartCounter(VirtualBasicTimer_t *pVirtualBasicTimerhandle){
	if (pVirtualBasicTimerhandle->status == VIRTUAL_BASIC_TIMER_ACTIVE){
		pVirtualBasicTimerhandle->start_counter_value = BasicTimer_GetCounter(&pVirtualBasicTimerhandle->realTimer);
	}
}
void VirtualBasicTimer_StopCounter(VirtualBasicTimer_t *pVirtualBasicTimerhandle){
	VirtualBasicTimer_Elapsed(pVirtualBasicTimerhandle);
	pVirtualBasicTimerhandle->counter_value_preStop = pVirtualBasicTimerhandle->elapsed_counter_value;
	pVirtualBasicTimerhandle->status = VIRTUAL_BASIC_TIMER_STOPPED;
}

/*Callback to application */
__attribute__((weak)) void VirtualBasicTimer_App_Callback(VirtualBasicTimer_t *pVirtualBasicTimerhandle){
    //This function will be implemented in every particular application. Thus, the weak attribute
}
