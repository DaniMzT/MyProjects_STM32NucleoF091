/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/** FSM RELATED **/
typedef enum {OFF_STATE, ON_STATE, EMERGENCY_STATE} MEF1_STATES; //known states (constants) of the MEF (Moore)
typedef uint8_t (*evaluateEvent) (void); //pointer to a function that evaluates an event
typedef struct {evaluateEvent Event; uint8_t nextState; } MEF_Transition; //transition defined by an event and next state after event evaluation
typedef void (*MEF_Action) (); //pointer to a function for the state actions (on entry/during/on exit
typedef struct {
	MEF_Action onEntry; //what to do on entry
	MEF_Action During; //what to do during the state
	MEF_Action onExit; //what to do just before exiting the state
	uint8_t numberTransitions;
	MEF_Transition *arrayTransitions;
}MEF_State; //state of an MEF
typedef struct {
	volatile uint8_t MEF_Active; //if MEF is active or not
	volatile uint8_t currentState; //MEF1_STATES or MEF2_STATES,etc.
	uint8_t numberStates; //number of states of the MEF
	volatile uint8_t flagStateChange; //if there has been a change of states
	MEF_State *arrayStates;
	//moved to MEF_State: MEF_Transition *arrayTransitions;
} MEF_MEF; //MEF

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** FSM RELATED **/
#define FSM_INACTIVE 0
#define FSM_ACTIVE 1
#define NUMBER_STATES 3
#define STATE_NOT_CHANGED 0
#define STATE_CHANGED 1
#define FALSE 0
#define TRUE 1
#define NUMBER_TRANSITIONS_OFF 3
#define NUMBER_TRANSITIONS_ON 3
#define NUMBER_TRANSITIONS_EMERGENCY 3

/*Temperature, I2C (ADS1115)*/
#define LENGTH_QUEUE_TEMP 2
#define ADS1115_ADDRESS (0x48) //0x48 is address of ADS1115 when ADDR connected to GND
#define ADS1115_CONVERSION_REGISTER_BYTES 2
#define ADS1115_GAIN 4076 //Gain amplifier in mV, check ADS1115 datasheet
#define ADS1115_MAX 32768 //15 bytes

//Group of event flags (bits) for the LCD_Arduino task
#define FLAG_STATUS_CHANGED (1<<0)
#define FLAG_STATION_CHANGED (1<<1)
#define FLAG_SPEED_CHANGED (1<<2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t uiFlagOnOff = FALSE; //handled by ButtonOnOff_Pin interrupt handler. button with spring return, not switch
uint8_t uiFlagEmergency = FALSE;//handled by EmergencyStopButton_Pin interrupt handler. button with spring return, not switch

//Group of event flags (bits) for the LCD_Arduino task
EventGroupHandle_t xGroupFlagsLCDArduino;
EventBits_t uxBitsFlagsLCDArduino;

//Temperature(I2C). Receive an item from a queue. The item is received by copy so a buffer of adequate size must be provided.
QueueHandle_t xQueue_Temperature = NULL;
volatile uint16_t uiBuffer_QueueTemperature;
uint8_t uiBytes_to_ADS1115_reg[3] = {0x00,0x00,0x00}; //for ADS1115 configuration (it can require 3 bytes)
uint8_t uiBytes_from_ADS1115[2] = {0x00,0x00}; //data received from ADS1115 (conversion register)
volatile uint16_t uiLastADS1115_raw = 0;
volatile uint16_t uiLastADS1115_mV = 0;
uint16_t uiADS1115_address = ADS1115_ADDRESS<<1; //STM32 HAL for I2C transmit/receive requires left shift
volatile uint8_t uiStep = 0; //needed for transmission/reception if we use IT functions,because they return OK after every byte sent and calls seem to be overwritten

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/** FSM functions **/
// Related to states: onEntry, during, onExit
void onEntry_Off();
void during_Off();
void onExit_Off();
void onEntry_On();
void during_On();
void onExit_On();
void onEntry_Emergency();
void during_Emergency();
void onExit_Emergency();
// Evaluate events for transitions
uint8_t evaluate_Event_Off_to_On();
uint8_t evaluate_Event_Off_to_Emergency();
uint8_t evaluate_Event_Off_Off();
uint8_t evaluate_Event_On_to_Off();
uint8_t evaluate_Event_On_to_Emergency();
uint8_t evaluate_Event_On_On();
uint8_t evaluate_Event_Emergency_to_Off();
uint8_t evaluate_Event_Emergency_to_On();
uint8_t evaluate_Event_Emergency_Emergency();

/** Task handlers **/
static void FSM_task_handler (void *parameters);
static void Motor_Control_task_handler (void *parameters);
static void Read_Temperature_task_handler (void *parameters);
static void LCD_Arduino_task_handler (void *parameters);

//Task handles, I call them pointers to avoid confussion with handlers (which are functions)
TaskHandle_t FSM_taskPointer;
TaskHandle_t Motor_Control_taskPointer;
TaskHandle_t Read_Temperature_taskPointer;
TaskHandle_t LCD_Arduino_taskPointer;

/*For the usage of printf via HAL_UART_Transmit*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Transitions
MEF_Transition OFF_TRANSITIONS[NUMBER_TRANSITIONS_OFF] = {
		  {evaluate_Event_Off_Off, OFF_STATE},
		  {evaluate_Event_Off_to_On, ON_STATE},
		  {evaluate_Event_Off_to_Emergency, EMERGENCY_STATE}
};
MEF_Transition ON_TRANSITIONS[NUMBER_TRANSITIONS_ON] = {
		  {evaluate_Event_On_On, ON_STATE},
		  {evaluate_Event_On_to_Off, OFF_STATE},
		  {evaluate_Event_On_to_Emergency, EMERGENCY_STATE}
};
MEF_Transition EMERGENCY_TRANSITIONS[NUMBER_TRANSITIONS_EMERGENCY] = {
		  {evaluate_Event_Emergency_Emergency, EMERGENCY_STATE},
		  {evaluate_Event_Emergency_to_Off, OFF_STATE},
		  {evaluate_Event_Emergency_to_On, ON_STATE}
};
//States
MEF_State MEF1_STATES_ARRAY[NUMBER_STATES] = {
		  {onEntry_Off, during_Off, onExit_Off, NUMBER_TRANSITIONS_OFF, OFF_TRANSITIONS},
		  {onEntry_On, during_On, onExit_On, NUMBER_TRANSITIONS_ON, ON_TRANSITIONS},
		  {onEntry_Emergency, during_Emergency, onExit_Emergency, NUMBER_TRANSITIONS_EMERGENCY, EMERGENCY_TRANSITIONS}
};
//FSM instances
MEF_MEF MEF1 = {FSM_ACTIVE, OFF_STATE, NUMBER_STATES, STATE_NOT_CHANGED, MEF1_STATES_ARRAY};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //Status from tasks (BaseType_t is the most efficient data type for the architecture;i.e.:32-bit for 32-bit architecture)
  BaseType_t task_Status;
  //Event of flags for LCD Arduino task
  xGroupFlagsLCDArduino = xEventGroupCreate();
  if (xGroupFlagsLCDArduino == NULL){
	  printf("Not possible to create xFlagsLCDArduino event group \n");
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //although I could turn on the blue LED (OFF state) in MX_GPIO_Init,I prefer to do call the OFF entry
  onEntry_Off();
  //I decide to set priorities higher than tskIDLE_PRIORITY.
  //FSM control highest prio as I want that OnOff/Emergency interrupts wakes the task
  task_Status = xTaskCreate(FSM_task_handler, "FSMcontrol", 200, NULL, tskIDLE_PRIORITY+2, &FSM_taskPointer);
  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging

  task_Status = xTaskCreate(Motor_Control_task_handler, "MotorControl", 200, NULL, tskIDLE_PRIORITY+1, &Motor_Control_taskPointer);
  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging

  //Create queue for reading temperature. if the queue is not created, do not create the task
  xQueue_Temperature = xQueueCreate(LENGTH_QUEUE_TEMP, sizeof(uint16_t));
  if (xQueue_Temperature != NULL){
	  task_Status = xTaskCreate(Read_Temperature_task_handler, "ReadTemperature", 200, NULL, tskIDLE_PRIORITY+1, &Read_Temperature_taskPointer);
	  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging
  }
  else{
	  printf("xQueue_Temperature failed to be created \n");
  }

  task_Status = xTaskCreate(LCD_Arduino_task_handler, "LCDArduino", 200, NULL, tskIDLE_PRIORITY+1, &LCD_Arduino_taskPointer);
  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging

  //Start scheduler
  vTaskStartScheduler();
  //Code should not reach here. vTaskStartScheduler must have failed due to insufficient heap memory?

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Green_Pin|LED_Blue_Pin|LED_Red_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Green_Pin LED_Blue_Pin LED_Red_Pin PA10 */
  GPIO_InitStruct.Pin = LED_Green_Pin|LED_Blue_Pin|LED_Red_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ButtonOnOff_Pin EmergencyStopButton_Pin */
  GPIO_InitStruct.Pin = ButtonOnOff_Pin|EmergencyStopButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
// Related to states: onEntry, during, onExit
void onEntry_Off(){
	//turn on blue LED, turn off the rest
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
}
void during_Off(){}
void onExit_Off(){}
void onEntry_On(){
	//turn on green LED, turn off the rest
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
}
void during_On(){}
void onExit_On(){}
void onEntry_Emergency(){
	//turn on red LED, turn off the rest
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);
}
void during_Emergency(){}
void onExit_Emergency(){}
// Evaluate events for transitions
uint8_t evaluate_Event_Off_to_On(){
	if (uiFlagOnOff && !uiFlagEmergency){
		//clear flag
		uiFlagOnOff = FALSE;
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_Off_to_Emergency(){
	if (uiFlagEmergency){
		//clear flag
		uiFlagEmergency = FALSE;
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_Off_Off(){
	if (!uiFlagOnOff && !uiFlagEmergency){//no change
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_On_to_Off(){
	if (uiFlagOnOff && !uiFlagEmergency){
		//clear flag
		uiFlagOnOff = FALSE;
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_On_to_Emergency(){
	if (uiFlagEmergency){
		//clear flag
		uiFlagEmergency = FALSE;
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_On_On(){
	if (!uiFlagOnOff && !uiFlagEmergency){ //no change
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_Emergency_to_Off(){
	//to undo emergency state,emergency button has to be pressed again. system can switch off in the middle of an emergency state
	if (uiFlagEmergency && uiFlagOnOff){
		//clear flags
		uiFlagEmergency = FALSE;
		uiFlagOnOff = FALSE;
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_Emergency_to_On(){
	//to undo emergency state,emergency button has to be pressed again.
	if (uiFlagEmergency){
		//clear flag
		uiFlagEmergency = FALSE;
		return TRUE;
	}
	return FALSE;
}
uint8_t evaluate_Event_Emergency_Emergency(){
	if (!uiFlagEmergency){ //no change
		return TRUE;
	}
	return FALSE;
}
/** Task handlers **/
static void FSM_task_handler (void *parameters){
	//task sleeps until an interrupt from on/off or emergency stop button happens
	//L-value not necessary <-- static uint32_t uiFSM_notification;

	//don't do 'static MEF_State *pState = MEF1.arrayStates' because In old C objects with static storage duration have to be initialized with constant expressions (literal constants)
	static MEF_State *pState; //initialization to NULL
	static MEF_Transition *pT;
	uint8_t uiTrans;
	for(;;){
		//waken up by a GPIO interrupt handler
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //uiFSM_notification = ... not necessary
		//Print
		printf("FSM\n");
		pState = MEF1.arrayStates;
		pT = pState[MEF1.currentState].arrayTransitions;
		//evaluate the state by iterating over the number of transitions of the current state
		for (uiTrans = 0; uiTrans < pState[MEF1.currentState].numberTransitions; uiTrans++){
			if (pT[uiTrans].Event() == TRUE){ //event true, transition activated
				// execute exit function of the state
				pState[MEF1.currentState].onExit();
				//assign next state, update the MEF and execute entry/do actions
				if (MEF1.currentState != pT[uiTrans].nextState) { //flag state changed
					MEF1.flagStateChange = STATE_CHANGED;
					MEF1.currentState = pT[uiTrans].nextState;
					//entry action of new state
					MEF1.arrayStates[MEF1.currentState].onEntry();
				}
				else{
					MEF1.flagStateChange = STATE_NOT_CHANGED;
					//do action
					MEF1.arrayStates[MEF1.currentState].During();
				}
				//connection with LCD_Arduino task
				uxBitsFlagsLCDArduino = xEventGroupSetBits(xGroupFlagsLCDArduino, (MEF1.flagStateChange)&FLAG_STATUS_CHANGED);
				//exit the loop
				break;
			}
		}

	}
}

static void Motor_Control_task_handler (void *parameters){
	//high priority if using real motor. if not real motor, based on SW timer

	for(;;){
		//Print
		//printf("Motor\n");
		//if state is off/emergency, speed target is 0. If on, it depends on the distance to the station
		switch(MEF1.currentState) {
		case OFF_STATE:
			//Print
			//printf(" OFF\n");
			break;
		case ON_STATE:
			//Print
			//printf(" ON\n");
			break;
		case EMERGENCY_STATE:
			//Print
			//printf(" EMER\n");
			break;
		default:
			//Print
			printf("Motor error\n");
			break;
		}
	}
}

static void Read_Temperature_task_handler (void *parameters){
	//periodic function. Block for 5000 ms. */
	const TickType_t xReadTemp_Delay = 5000 / portTICK_PERIOD_MS;

	for(;;){
		//Print
		printf("Temperature\n");
		//Configure I2C communication
		/*HAL_I2C_Master_Sequential_Transmit_IT:The device 7 bits address value in datasheet must be shifted to the left before calling the interface.
		*Returns almost immediately and lets the main program continue, while the I2C transfer is in progress.
		*That's why I'm using uiStep, in order not to overwrite calls */
		switch(uiStep){
		case 0:
			//0.1.Write to config reg (0x01 points to it)
			uiBytes_to_ADS1115_reg[0] = 0x01;
			//0.2.+-4.076V,128 sps,b14-12 110:INp=AIN0 AInn=GND (single ended meas A0).b15:begin single conversion.b8: 1-single shot
			uiBytes_to_ADS1115_reg[1] = 0xC3; //0b11000011 (MSB)
			uiBytes_to_ADS1115_reg[2] = 0x83; //0b10000011 (LSB)
			if (HAL_I2C_Master_Transmit_IT(&hi2c1, uiADS1115_address, uiBytes_to_ADS1115_reg, 3) != HAL_OK){
				printf("err1\n");
			}
			break;
		case 1://1.Select the conversion register by writing to pointer reg:address+0x00(points to conversion register)
			uiBytes_to_ADS1115_reg[0] = 0x00; //0x00(points to conversion register)
			if (HAL_I2C_Master_Transmit_IT(&hi2c1, uiADS1115_address, uiBytes_to_ADS1115_reg, 1) != HAL_OK){
				printf("err2\n");
			}
			break;
		case 2: //2.Reception
			if (HAL_I2C_Master_Receive_IT(&hi2c1, uiADS1115_address, uiBytes_from_ADS1115, ADS1115_CONVERSION_REGISTER_BYTES) != HAL_OK){
				printf("err3\n");
			}
			//Wait for reception from queue.Block for 10 ticks if a message is not immediately available.later on,restart transmission
			if(xQueueReceive( xQueue_Temperature, (void *)&uiBuffer_QueueTemperature, (TickType_t) 10 ) == pdPASS )
			{
				//uiBuffer_QueueTemperature now contains a copy of xQueue_Temperature
				printf(":%d\n", uiBuffer_QueueTemperature);
			}
			else{
				printf("EmptyQ\n");
			}
			break;
		default://error
			printf("NoCase\n");
			uiStep = 0;//start transmission -> uiStep=0
			break;
		}

		//Block the task for a period of ticks
		vTaskDelay(xReadTemp_Delay);
	}
}

static void LCD_Arduino_task_handler (void *parameters){
	//Task blocked until a flag of the event group is active
	//I want to wait forever,leaving blocked state only when unblock condition happens.const TickType_t xTicksToWait_LCDArduino = 1/portTICK_PERIOD_MS; //1 ms

	for(;;){
		//Print
		printf("LCD\n");
		//wait for any flag active ( xWaitForAllBits=FALSE).Clear the active flags ( xClearOnExit=pdTRUE)
		//uxBitsFlagsLCDArduino = xEventGroupWaitBits(xGroupFlagsLCDArduino, FLAG_SPEED_CHANGED|FLAG_STATION_CHANGED|FLAG_STATUS_CHANGED, pdTRUE, pdFALSE, xTicksToWait_LCDArduino);
		uxBitsFlagsLCDArduino = xEventGroupWaitBits(xGroupFlagsLCDArduino, FLAG_SPEED_CHANGED|FLAG_STATION_CHANGED|FLAG_STATUS_CHANGED, pdTRUE, pdFALSE, portMAX_DELAY);
		//enable SPI

		//send to Arduino: state + station + temperature

		//disable SPI
	}
}

//EXTI interrupt handler (OnOff button or emergency stop). __weak void overwritten
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//from OnOff button
	if (GPIO_Pin == ButtonOnOff_Pin){
		//set/reset flag
		uiFlagOnOff = TRUE;
		//notify task FSM

	}
	//from emergency stop
	if (GPIO_Pin == EmergencyStopButton_Pin){
		//set/reset flag
		uiFlagEmergency = TRUE;
		//notify task FSM

	}
	// Notify the thread so it will wake up when the ISR is complete
	vTaskNotifyGiveFromISR(FSM_taskPointer, &xHigherPriorityTaskWoken);
	/*Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE. If vTaskNotifyGiveFromISR indicates that a higher priority task is being woken,
	 *portYIELD_FROM_ISR() routine will context switch to that task after returning from the ISR.Failure to use this function will result
	 *in execution resuming at previous point rather than switching to new context*/
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//I2C reception handler callback. __weak void overwritten
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c){
	if (hi2c->Instance == hi2c1.Instance){ //I2C1
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		uiLastADS1115_raw = (uiBytes_from_ADS1115[0] << 8)|(uiBytes_from_ADS1115[1]); //byteRead is LSB and firstByteRead is MSB
		uiLastADS1115_mV = uiLastADS1115_raw * ADS1115_GAIN/ADS1115_MAX;
		xQueueSendFromISR(xQueue_Temperature, (void *)&uiLastADS1115_mV, &xHigherPriorityTaskWoken);
		//switch context if necessary. */
		if( xHigherPriorityTaskWoken )
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		uiStep++;
		uiStep%=3;
	}

}

//I2C transmission handler callback. __weak void overwritten
void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c){
	if (hi2c->Instance == hi2c1.Instance){ //I2C1
		uiStep++;
		uiStep%=3;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
