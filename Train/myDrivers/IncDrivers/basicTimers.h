/*
 * basicTimers.h
 *
 *  Created on: Dec 28, 2022
 *      Author: danimt
 */

#ifndef INCDRIVERS_BASICTIMERS_H_
#define INCDRIVERS_BASICTIMERS_H_

#include "stm32f091rct6.h"

/***********************************STRUCTS****************************************************/
/*structure to configure the basic timers, which will be linked to the registers(BasicTimer_RegStruct_t in stm32f091rct6.h) in the driver functions*/
typedef struct{
    uint8_t BasicTimer_arpe; //0:TIM_ARR not buffered (when ARR is changed,the update event will be adapted to new change)
    uint8_t BasicTimer_opm; //0:counter not stopped at next event;1:stopped at next event (and CEN=0)
    uint8_t BasicTimer_urs; //1:interrupt only with overflow/underflow counter; 0:also other interrupt sources
    uint8_t BasicTimer_udis; //0:Update Event (UEV) enabled; 1:The Update event is not generated, shadow registers keep their value (ARR, PSC)
    uint16_t BasicTimer_prescaler;
    uint16_t BasicTimer_autoreload;
}BasicTimer_Config_t;

/*structure to handle the basic timers: configuration struct + pointer to registers(BasicTimer_RegStruct_t in stm32..h)*/
typedef struct{
    BasicTimer_RegStruct_t *pBasicTimer;
    BasicTimer_Config_t BasicTimer_Config;
}BasicTimer_Handle_t;

/*structure for virtual timers,when there are no physical timers.They depend on a basic timer counter and are used in polling mode of main.c
 * Their counters are supposed to be shorter than the associated basic timer;otherwise,modify VirtualBasicTimer_Elapsed*/
typedef struct{
    BasicTimer_Handle_t realTimer; //real timer to which the virtual timer is linked
    uint16_t configured_counter_value;
    uint16_t start_counter_value; //real timer value when virtual counter starts
    int16_t elapsed_counter_value; //it can be negative if real timer counter has been updated
    uint16_t counter_value_preStop; //virtual counter value before virtual counter stopped
    uint8_t status; //active, inactive or stopped
}VirtualBasicTimer_t;

/*Macros and enums*/
enum VirtualBasicTimer_Status {VIRTUAL_BASIC_TIMER_INACTIVE,VIRTUAL_BASIC_TIMER_ACTIVE,VIRTUAL_BASIC_TIMER_STOPPED};
enum VirtualBasicTimer_IntervalStatus {VIRTUAL_BASIC_TIMER_NOT_ELAPSED,VIRTUAL_BASIC_TIMER_ELAPSED,FUNCTION_NOT_ACCESSIBLE};

/***********************************FUNCTIONS****************************************************/
/*Enable or disable BasicTimer peripheral clock*/
void BasicTimer_ClockControl(BasicTimer_Handle_t *pBasicTimerhandle, uint8_t EnableDisable);

/*BasicTimer initialization*/
void BasicTimer_Init(BasicTimer_Handle_t *pBasicTimerhandle);

/*BasicTimer control: enable/disable --> CEN */
void BasicTimer_EnableDisable(BasicTimer_Handle_t *pBasicTimerhandle, uint8_t EnableDisable);

/*Reset BasicTimer */
void BasicTimer_Reset(BasicTimer_Handle_t *pBasicTimerhandle);

/*Get the counter value*/
uint16_t BasicTimer_GetCounter(BasicTimer_Handle_t *pBasicTimerhandle);

/*Restart counter*/
void BasicTimer_RestartCounter(BasicTimer_Handle_t *pBasicTimerhandle);

/*IRQ Configuration and ISR handling*/
void BasicTimer_IRQ_EnableDisable(uint8_t IRQ_Number, uint8_t EnableDisable);
void BasicTimer_IRQ_Priority(uint8_t IRQ_Number, uint32_t IRQ_Priority);
void BasicTimer_IRQ_Handling(BasicTimer_Handle_t *pBasicTimerhandle);

/*Callback*/
void BasicTimer_App_Callback(BasicTimer_Handle_t *pBasicTimerhandle);

/*Update disable*/
void BasicTimer_UDIS(BasicTimer_Handle_t *pBasicTimerhandle, uint8_t EnableDisable); //UDIS=update disable-> 0:UEV enabled; 1:UEV disabled

/*Static functions, interrupt handlers*/
/*NOT NEEDED AS THERE IS ONLY ONE INTERRUPT BIT
static void BasicTimer_UpdateInterruptFlag_handler(BasicTimer_Handle_t *pBasicTimerhandle);
*/

/*Virtual basic timer functions*/
enum VirtualBasicTimer_IntervalStatus VirtualBasicTimer_Elapsed(VirtualBasicTimer_t *pVirtualBasicTimerhandle);
void VirtualBasicTimer_EnableDisable(VirtualBasicTimer_t *pVirtualBasicTimerhandle, uint8_t EnableDisable);
void VirtualBasicTimer_StartCounter(VirtualBasicTimer_t *pVirtualBasicTimerhandle);
void VirtualBasicTimer_App_Callback(VirtualBasicTimer_t *pVirtualBasicTimerhandle);
void VirtualBasicTimer_StopCounter(VirtualBasicTimer_t *pVirtualBasicTimerhandle);

#endif /* INCDRIVERS_BASICTIMERS_H_ */
