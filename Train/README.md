# MyProjects_STM32-Nucleo-F091RC
Personal projects on a STM32 Nucleo-F091RC

TRAIN:
The target is to simulate a train with a main finite-state machine which controls train motion by actuating on a step motor according to the inputs received.
Inputs: on/off switch and emergency button (simulating an issue, like doors not closing), temperature sensor
Outputs: Step motor, fan, LEDs to indicate movement/stop/emergency, LCD 1602A to indicate station and speed
Modules before final work: 
1) GPIOs on STM32: LEDs (for example, to display on/off/emergency) and switches (on/off/emergency)
2) 1+SPI through Arduino (STM32 master, Arduino slave --> STM32 sending status)
option A (3A, 4A)
3A) 2+LCD 1602A connected to Arduino so that it displays what STM32 sends (add station in the message?)
4A) Temperature sensor connected to ADC, which communicates with STM32 through I2C
option B (3B, 4B)
3B) like 2 but connecting a temperature sensor to Arduino, so it will send temperature to STM32 via SPI when STM32 requests that
4B) LCD display via I2C from STM32
5) Controlling step motor through STM32

