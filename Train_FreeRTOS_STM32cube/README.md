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
- FSMcontrol (Moore's machine,output depends on the event).management of state, which is a parameter in many tasks. actions on entry/during/on exit as short as possible if can be delegated on tasks)
- Motor_Control (when motor available; otherwise, SW timer)
- Read_Temperature
- Send_Arduino_SPI (depends on the state)
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
--
TESTS
- emergency, on, off tasks only run after interrupts, never periodically
- simulate emergency stop during off or on tasks (make both long enough). emergency must take over them and on/off tasks not resumed afterwards.
- simulate off/on during emergency task (make it long enough). on/off can't take over emergency

IDEAS 
Palancas emergencia, semaforos/señales stop externos
Hacerlo robusto con manejo errores posibles. waitForever o timeout, depende de si repetir iteracion en caso de no obtener valor cola nos interesa o no. 
Para mensajes comunicacion, no printf. ver opciones que explican primeros videos ( SWO SWV ITM data console con task_Action(solo STM32 CSIS?)...)
Semaforo binario/mutex: acceso freno (emergencia?)/motor
Semáforo contador: nºestación. avance derecha suma, avance izquierda resta. pensar también en algun recurso limitado. Hay alguna manera de saber valor contador semaforo y adquirir en base a un numero concreto? por ejemplo, que cuando semaforo==2, adquirir.
algun event flag para que una tarea se ejecute tras una combinacion de eventos (de una interrupcion + otra tarea, p.ej).
task notifications: habilitar motor o freno desde tareas, añadiendo motor y freno como mutex.

SW interrupts:
The syntax used to generate a software interrupt is dependent on the
FreeRTOS port being used. The syntax used below can only be used with
the FreeRTOS Windows port, in which such interrupts are only simulated. */
vPrintString( "Periodic task - About to generate an interrupt.\r\n" );
vPortGenerateSimulatedInterrupt( mainINTERRUPT_NUMBER );
vPrintString( "Periodic task - Interrupt generated.\r\n\r\n\r\n" );

