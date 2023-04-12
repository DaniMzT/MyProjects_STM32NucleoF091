#Train_FreeRTOS
This project modifies Train by using FreeRTOS as operating system, instead of being bare metal. However, CMSIS layer is not used for FreeRTOS (I deal with pure FreeRTOS to make it independent from the uC model). Regarding management with hardware, STM32Cube functions are used (I2C, SPI, etc).

1) Train_FreeRTOS_simMove
No motor is used, the move is simulated by SW timer. As in bare metal, on/off switches and an emergency stop button are used as GPIOs. A temperature sensor is connected via I2C.

APPROACH WITH FSM
States running on task "FSMcontrol"
States:
- Motion
- Turn_Off
- Emergency
Tasks:
- FSMcontrol (Moore's machine,output depends on the event).management of state, which is a parameter in many tasks. actions on entry/during/on exit as short as possible if can be delegated on tasks).
Interrupts from on/off switches and emergency stop button preempt task FSMcontrol. These interrupts enable/disable some flags, which will be used to evaluate states.
- Motor_Control (when motor available; otherwise, SW timer)
- Read_Temperature (periodic)
- Send_Arduino_SPI (active only if a bit of the next event group is active: state changed/new station/speed change)
- LEDs_control (depends on the state).It could be an on entry action of every state, but as it's common for the states I prefer to use it as a task

APPROACH WITHOUT FSM
Tasks:
NOTE: Tasks in the Suspended state are not available to the scheduler.the only way out being through a call to the vTaskResume() or xTaskResumeFromISR()
- Start_Move: 
	Only runs after button 'on' activation (interrupt calls xTaskResumeFromISR()). So at the beginning of task code suspend/block (TaskNotifyTake; TaskNotifyGive from related ISR)
	If possible, when task is resumed, restart it. Otherwise, make task as short as possible/check global flag (continue = next iteration)
	Lower priority than Turn_Off and Emergency -> better through interrupt priorities which trigger Start_Move/Turn_Off/Emergency
	Change state to ON. if already ON, do not run (based on EventBits_t in event groups?)
	Actuate motor (if virtual move: start/resume SW timer; if real motor: call function Motor_Control to accelerate)
	Call function Send_Arduino_SPI
	Wake up task Motion (task notification)
- Turn_Off:
	Only runs after button 'off' activation (interrupt calls xTaskResumeFromISR() ). So at the beginning of task code suspend/block (TaskNotifyTake; TaskNotifyGive from related ISR)
	If possible, when task is resumed, restart it. Otherwise, make task as short as possible/check global flag (continue = next iteration)
	Higher priority than Start_Move but lower than Emergency. -> better through interrupt priorities which trigger Start_Move/Turn_Off/Emergency
	Change state to OFF. if already OFF, do not run (based on EventBits_t in event groups?)
	Actuate motor (if virtual move: stop SW timer; if real motor: slow down)
	Call function Send_Arduino_SPI
- Emergency:
	Only runs after emergency stop (interrupt calls xTaskResumeFromISR() ) or too hot temperature (vTaskResume() from task Read_Temperature). So at the beginning of task code suspend/block (TaskNotifyTake; TaskNotifyGive from related ISR)
	Critical section during execution?
	Highest priority. -> better through interrupt priorities which trigger Start_Move/Turn_Off/Emergency
	Change state to EMERGENCY. if already EMERGENCY, do not run (based on EventBits_t in event groups?)
	Actuate motor
	If notification from Read_Temperature due to too hot temperature, cool down (function/task TBD)
	Call function Send_Arduino_SPI
	Task finishes as long as emergency stop not active and temperature is ok 
- Motion:
	Only runs if state ON
	Periodic. Calls Motor_Control, which adjusts speed according to current speed and distance to the station
- Read_Temperature:
	Runs periodically enabling I2C reading
	Read from a queue
	If temperature is too high, wake up Emergency (vTaskResume())
	
Functions:
- Motor_Control(input parameter depending on the caller task -> accelerate(Start_Move)/soft brake(Turn_Off)/hard brake(Emergency)/any of previous depending on distance to station/current speed(Motion)  (for virtual move, activate/stop software timer)
- I2C related
- Send_Arduino_SPI: (alternative: gatekeeper task?)
	message is a static variable, reset in every call to avoid corruption between tasks. is a mutex necessary here, like example 20, unit 7 from manual?
	enable SPI
	send to Arduino: state + station + temperature
	disable SPI
- Software timers for virtual move:
	increase/decrease station number depending on direction
	Call function Send_Arduino_SPI

Interrupts:
- Button on: lower priority than button 'off' and emergency stop
- Button off: lower priority than emergency stop
- Emergency stop: high priority
	portYIELD_FROM_ISR(), ensuring the ISR returns directly to Emergency task (read p201 manual, how to use xHigherPriorityTaskWoken)
- Temperature sensor via ADC (I2C): less priority. Pushes back to temperature queue
- Arduino communication for LCD (SPI): less priority

