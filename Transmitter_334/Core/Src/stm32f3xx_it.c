/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
//#include "meas.h."
//#include "qi_transmitter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define ANALOG_PING_DURATION 14 // 14 period pri 200 kHz... 70 us
#define BIT_PERIOD_US 500
#define ONE_MIN_US 120 //predtim 160
#define ONE_MIN_FIRST_US 70 // predtim 80
#define ONE_MAX_US 360 //350 // mozna i o neco vic lehce... videl jsem i 360 us
#define ONE_MAX_SECOND_US 490 // 480 fungovalo celkem pekne
#define ZERO_MIN_US 361
#define ZERO_MAX_US 750 //rpredtim 800
#define ONE_MIN_TICKS ONE_MIN_US * 36 //...36000000/1000000 HRTIM CLOCK is 144MHz, prescaler 4... 144Mhz/4..ticks freq. is 36Mhz
#define ONE_MAX_TICKS ONE_MAX_US * 36
#define ZERO_MIN_TICKS ZERO_MIN_US * 36
#define ZERO_MAX_TICKS ZERO_MAX_US * 36
#define ONE_MIN_FIRST_TICKS ONE_MIN_FIRST_US * 36
#define ONE_MAX_SECOND_TICKS ONE_MAX_SECOND_US * 36
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern int systick_counter;
extern bool one_ms;
extern bool new_FSK;
//extern bool first_half_FSK;
//extern bool* FSK_byte;
extern bool FSK_interrupt;
//extern unsigned int FSK_index;
extern bool new_dbg_uart_msg;
extern void ModulateFrequency(void);
uint32_t capture_value;
extern bool comp_treshold;
extern bool analog_ping;
extern bool analog_ping_done;
extern char analog_ping_counter;
extern bool analog_ping_measure;
bool false_edge=false;
bool one_flag=false;
uint32_t memory = 0;
//bool ASK_buffer[11];
uint16_t ASK_byte;
extern uint16_t ASK_byte_main;
uint8_t ASK_byte_position=0;
bool prev_done=false; //to check wether it was decided about the bit to be ZERO or ONE in the previous edge
bool ASK_error=false;
bool preamble_done=false;
bool ASK_byte_done=false;
uint8_t preamble_ones=0;
//bool ASK_byte[11];
extern uint8_t ASK_msg_position;
extern uint8_t current_ASK_msg_size;
extern uint8_t error_type;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern COMP_HandleTypeDef hcomp2;
extern DAC_HandleTypeDef hdac2;
extern HRTIM_HandleTypeDef hhrtim1;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	one_ms=true;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
	//new_dbg_uart_msg=true;
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXT line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global and DAC1 underrun error interrupts.
  */
void TIM6_DAC1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC1_IRQn 0 */

  /* USER CODE END TIM6_DAC1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC1_IRQn 1 */

  /* USER CODE END TIM6_DAC1_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global and DAC2 underrun error interrupts.
  */
void TIM7_DAC2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC2_IRQn 0 */

  /* USER CODE END TIM7_DAC2_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac2);
  /* USER CODE BEGIN TIM7_DAC2_IRQn 1 */

  /* USER CODE END TIM7_DAC2_IRQn 1 */
}

/**
  * @brief This function handles COMP2 global interrupt through EXTI line 22.
  */
void COMP2_IRQHandler(void)
{
  /* USER CODE BEGIN COMP2_IRQn 0 */
	
  /* USER CODE END COMP2_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp2);
  /* USER CODE BEGIN COMP2_IRQn 1 */
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	comp_treshold = true;
  /* USER CODE END COMP2_IRQn 1 */
}

/**
  * @brief This function handles HRTIM timer A global interrupt.
  */
void HRTIM1_TIMA_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 0 */
	capture_value = HAL_HRTIM_GetCapturedValue(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A,HRTIM_CAPTUREUNIT_1);
  /* USER CODE END HRTIM1_TIMA_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A);
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 1 */
	//TODO: PREAMBLE!!!!
	if (false_edge){
		capture_value+=memory;
		false_edge=false;
	}
	if (((ONE_MIN_TICKS<capture_value&&capture_value<ONE_MAX_SECOND_TICKS)&&one_flag)||((ONE_MIN_FIRST_TICKS<capture_value&&capture_value<ONE_MAX_TICKS)&&(!one_flag))){
		if (one_flag){
			if (preamble_done){
				//ASK_buffer[ASK_byte_position]=true;
				ASK_byte |= (1 << ASK_byte_position); //stores 1 on current position
				//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
				//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
				ASK_byte_position++;
				prev_done=true;
			}else{
				preamble_ones++;
			}
			one_flag=false;
		}else{
			one_flag=true; //tady asi
		}
		memory=capture_value; //ZJISTIT, jestli to nem� bejt jenom u else vej�
	}
	else if (ZERO_MIN_TICKS<capture_value&&capture_value<ZERO_MAX_TICKS){
		if (one_flag){
			one_flag=false;
		}
		if (preamble_done){
			//ASK_buffer[ASK_byte_position]=false;
			//ASK_byte &= ~(1 << ASK_byte_position); //stores 0 on current position 
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
			//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			ASK_byte_position++;
			prev_done=true;
			memory=capture_value;
		}else{
			if (preamble_ones>=4){ //min 4 jednicky
				preamble_done=true;
				//ASK_buffer[ASK_byte_position]=false;
				//ASK_byte &= ~(1 << ASK_byte_position);
				//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
				//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);	
				ASK_byte_position=0;				
				ASK_byte_position++;
				preamble_ones=0;
				ASK_byte=0;
				
				//TODO:po checksum hodit preamble_done do false
			}else{
				ASK_error = true;
				error_type=5; // 0 v ramci preambuli
			}
		}
	}
	else if (capture_value<ONE_MIN_TICKS){
		false_edge=true;
		memory+=capture_value;
		if (prev_done){
			ASK_byte_position--;
			prev_done=false;
		}
	} 
	else {
		ASK_error=true;
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
		error_type=6;		
	}
	if (ASK_byte_position==11){
		/*for (int i = 0;i<11;i++){
			ASK_byte[i]=ASK_buffer[i];
		}*/
		ASK_byte_main = ASK_byte;
		ASK_byte = 0x0000;
		ASK_byte_position=0;
		ASK_msg_position++;
		ASK_byte_done=true;
		if (ASK_msg_position==3){//current_ASK_msg_size){ //length of CE data packet
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			preamble_done=false;
		}
	}
	if (ASK_error){
		//ASK_byte=0x0000;
		//preamble_ones=0;
		//preamble_done=false;
		//ASK_byte_position = 0;
		//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
	}
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
  
  /* USER CODE END HRTIM1_TIMA_IRQn 1 */
}

/**
  * @brief This function handles HRTIM timer C global interrupt.
  */
void HRTIM1_TIMC_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMC_IRQn 0 */

  /* USER CODE END HRTIM1_TIMC_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_C);
  /* USER CODE BEGIN HRTIM1_TIMC_IRQn 1 */
  if (new_FSK){
	FSK_interrupt=true;
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);	
  }
	
  /* USER CODE END HRTIM1_TIMC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
