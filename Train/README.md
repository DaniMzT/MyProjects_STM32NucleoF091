# MyProjects_STM32-Nucleo-F091RC
Personal projects on a STM32 Nucleo-F091RC

TRAIN:
The target is to simulate a train with a main finite-state machine which controls train motion by actuating on a step motor according to the inputs received.
Inputs: on/off switch and emergency button (simulating an issue, like doors not closing), temperature sensor (Arduino through SPI)
Outputs: Step motor, fan, LEDs to indicate movement/stop/emergency, LCD 1602A to indicate station and speed (I2C)
Modules before final work: 
1) GPIOs on STM32: LEDs (for example, to display on/off/emergency) and switches (on/off/emergency)
2) 1)+SPI through Arduino (STM32 master, Arduino slave --> STM32 sending status, but in the final project STM32 will request temperature, sent by Arduino)
3) LCD display via I2C. STM32 sends station, speed and temperature (Arduino now sending temperature via SPI)
4) Controlling step motor through STM32

