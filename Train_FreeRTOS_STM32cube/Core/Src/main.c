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
#include "FreeRTOS.h"
#include "task.h"

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
		  {onEntry_On, during_On, onExit_On, NUMBER_TRANSITIONS_ON}, ON_TRANSITIONS,
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
	//Task handles, I call them pointers to avoid confussion with handlers (which are functions)
	TaskHandle_t FSM_taskPointer;
	TaskHandle_t Motor_Control_taskPointer;
	TaskHandle_t Read_Temperature_taskPointer;
	TaskHandle_t LCD_Arduino_taskPointer;
	//Status from tasks (BaseType_t is the most efficient data type for the architecture;i.e.:32-bit for 32-bit architecture)
	BaseType_t task_Status;

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
  /* USER CODE BEGIN 2 */
  task_Status = xTaskCreate(FSM_task_handler, "FSMcontrol", 200, NULL, 2, &FSM_taskPointer);
  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging
  task_Status = xTaskCreate(Motor_Control_task_handler, "FSMcontrol", 200, NULL, 3, &Motor_Control_taskPointer);
  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging
  task_Status = xTaskCreate(Read_Temperature_task_handler, "FSMcontrol", 200, NULL, 2, &Read_Temperature_taskPointer);
  configASSERT(task_Status == pdPASS); //if false, then infinite loop. good for debugging
  task_Status = xTaskCreate(LCD_Arduino_task_handler, "FSMcontrol", 200, NULL, 1, &LCD_Arduino_taskPointer);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Related to states: onEntry, during, onExit
void onEntry_Off(){};
void during_Off(){};
void onExit_Off(){};
void onEntry_On(){};
void during_On(){};
void onExit_On(){};
void onEntry_Emergency(){};
void during_Emergency(){};
void onExit_Emergency(){};
// Evaluate events for transitions
uint8_t evaluate_Event_Off_to_On(){
	return FALSE;
};
uint8_t evaluate_Event_Off_to_Emergency(){
	return FALSE;
};
uint8_t evaluate_Event_Off_Off(){
	return FALSE;
};
uint8_t evaluate_Event_On_to_Off(){
	return FALSE;
};
uint8_t evaluate_Event_On_to_Emergency(){
	return FALSE;
};
uint8_t evaluate_Event_On_On(){
	return FALSE;
};
uint8_t evaluate_Event_Emergency_to_Off(){
	return FALSE;
};
uint8_t evaluate_Event_Emergency_to_On(){
	return FALSE;
};
uint8_t evaluate_Event_Emergency_Emergency(){
	return FALSE;
};
/** Task handlers **/
static void FSM_task_handler (void *parameters){
	//only active if an interrupt from on/off or emergency stop button happens
	static uint8_t *pState = MEF1->arrayStates;
	static MEF_Transition *pT = pState[MEF1->currentState]->arrayTransitions;
	uint8_t uiTrans;
	for(;;){
		//evaluate the state by iterating over the number of transitions of the current state
		for (uiTrans = 0; uiTrans < pState[MEF1->currentState]->numberTransitions; uiTrans++){
			if (pT[uiTrans]->Event() == TRUE){ //event true, transition activated
				// execute exit function of the state
				pState[MEF1->currentState].onExit();
				//assign next state, update the MEF and execute entry/do actions
				if (MEF1->currentState == pT[uiTrans]->nextState) { //flag state changed
					MEF1->flagStateChange = STATE_CHANGED;
					//entry action
					MEF1->arrayStates[MEF1->currentState].onEntry();
				}
				else{
					MEF1->flagStateChange = STATE_NOT_CHANGED;
					//do action
					MEF1->arrayStates[MEF1->currentState].During();
				}
				MEF1->currentState = pT[uiTrans]->nextState;
			}
		}

	}
}

static void Motor_Control_task_handler (void *parameters){
	//high priority if using real motor. if not real motor, based on SW timer
	for(;;){

	}
}

static void Read_Temperature_task_handler (void *parameters){
	//high priority, periodic function
	for(;;){

	}
}

static void LCD_Arduino_task_handler (void *parameters){

	for(;;){
		//enable SPI

		//send to Arduino: state + station + temperature

		//disable SPI
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
