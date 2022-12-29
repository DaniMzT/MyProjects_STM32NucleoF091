# MyProjects_STM32-Nucleo-F091RC
Personal projects on a STM32 Nucleo-F091RC

TRAIN:
The target is to simulate a train with a main finite-state machine which controls train motion by actuating on a step motor according to the inputs received.
Inputs: on/off switch and emergency button (simulating an issue, like doors not closing), temperature sensor
Outputs: Step motor, fan, LEDs to indicate movement/stop/emergency, LCD 1602A to indicate station and speed

Modules: 
1) GPIOs on STM32: LEDs (for example, to display on/off/emergency) and switches (on/off/emergency)
Name: trainButtonsLedsInterrupts.c

2) 1+SPI through Arduino (STM32 master, Arduino slave --> STM32 sending status)
Name: trainSPI_Arduino_NOdebug.c

3) 1+I2C to acquire temperature (ADC is ADS1115)
Name: train_I2C.c

4) 2+3+LCD 1602A connected to Arduino via SPI so that it displays a message with the station and the state whenever either of both changes. The stations are simulated
by means of a virtual timer, while I2C measurements are acquired periodically based on a basic timer.
Name: train_LCD_timers.c

5) Separated module to control step motor through STM32

6) Final: 4+5. Now the train has real motion, so now stations are not simulated by a virtual timer. Build everything according to a Finite-States Machine.

