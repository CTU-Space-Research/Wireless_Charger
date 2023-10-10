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
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "qi_transmitter.h"
#include "stdio.h"
#include "string.h"
#include "meas.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {float op_frequency;float op_period;uint32_t op_period_in_ticks; uint32_t mod_period_in_ticks;bool gate_signals;}op_values;
enum OPERATION_STATES { INITIAL_STATE = 1, ANALOG_PING = 2, DIGITAL_PING = 3, COMM_ONLY_STATE = 4, RENEGO_STATE = 5, POWER_TRANSFER_STATE = 6, FAULT_STATE = 7 };


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HRTIM_RESOLUTION 217 //in [ps], used in ChangeFrequency
//#define TICKS_DELTA_MODULATION (uint32_t) (260000/HRTIM_RESOLUTION) //260 ns
#define TICKS_DELTA_MODULATION 375 //750//nejlip 1500//1200 //10000 //1200 //test value --> 260 ns
#define ANALOG_PING_FREQUENCY 170//192 // kHz --> freq. corresponding to the min. power level
#define DBG_MSG_SIZE 1
#define MAX_FREQUENCY 235.97//200 // 5us
#define PING_IMP_FREQUENCY 228.97
#define MIN_FREQUENCY 160.26//168
#define DAC_TRESHOLD  2900u//2680u //odpovídá 1.2 A
#define DBG_UART_MSG_SIZE 88

#define LED1 GPIO_PIN_5 //B
#define LED2 GPIO_PIN_4 //B
#define LED3 GPIO_PIN_3 //B
#define LED4 GPIO_PIN_2 //B
#define PID_ITERATIONS 5
#define T_DELAY 100
#define T_DETECT 70
#define T_REPING 1500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t hal_tick;
bool one_ms;
unsigned int test_increment;
bool new_dbg_uart_msg;
//unsigned char FSK_index;

//char DBG_UART_buffer[50];
//char DBG_UART_buffer[100];


//ADC
uint8_t MEAS_ACD1_update;
//extern float adcf[MEAS_ADC1_CHANNELS];  //---------------------ZBYTECNE
//uint8_t ubAnalogWatchdogStatus = RESET;
measured_values act_vals;
//DAC
bool comp_treshold = false;
//test adc
bool run_test = false;
//mereni
//float current_values[NUMBER_OF_MES];  //---------------------ZBYTECNE
//analog ping 
bool analog_ping_done = false;
char analog_ping_counter = 0;
bool analog_ping_measure = false;
//char analog_ping_counter = 0;
float K_p=9.;//37.;
float K_i=3.; //6.;
float K_d=0;
uint8_t current_ASK_msg_size=3; //minimal size
uint8_t ASK_msg_position=0;
uint16_t ASK_byte_main;
uint8_t error_type=0;
bool FSK_interrupt=false;
bool new_FSK=false;
uint16_t FSK_size;
bool serialized_data_FSK[MAX_DATA_PACKET_SIZE_FSK];
uint8_t last_message=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ChangeOPFrequency(float frequency,op_values *current_op); // function that changes the frequency of HRTIM output
static void StartInverter(op_values *current_op);
static void StopInverter(op_values *current_op);
void ModulateFrequency(op_values *current_op);
void AnalogPing(op_values *current_op);
void StartDemodulator(void);
//float regulatePower(uint8_t u_control_error,op_values *current_op);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Rx_data[1];	
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
 // HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_3);  // toggle PA0
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  HAL_UART_Receive_DMA(&huart1, Rx_data, DBG_MSG_SIZE);
	new_dbg_uart_msg=true;	
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6 && analog_ping_measure)
  {
    // Toggle LED
    if (analog_ping_counter==1)
		{
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			MEAS_ADC_start(5u,0);
			
			//HAL_TIM_Base_Stop_IT(&htim6);
			
		} else if (analog_ping_counter==14){
			analog_ping_done=true;
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(&htim6);
			//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
			analog_ping_measure=false;
		}
		analog_ping_counter++;
		
  }
}
//void HAL_TIM_Base_Start_IT()
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
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
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_COMP2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	op_values current_op;
	//DAC
	//HAL_DAC_Start(&hdac2, DAC_CHANNEL_1); //HAZELO TO FURT ERRORY
	//HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R,DAC_TRESHOLD); //HAZELO TO FURT ERRORY
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* Start the SYSTICK variable*/
	//hal_tick = HAL_GetTick();
	test_increment=0;
	one_ms=true;
	unsigned int number_of_ms=0;
	bool button = false;
	char button_increment=0;
	new_dbg_uart_msg = false;
	HAL_UART_Receive_DMA (&huart1, Rx_data, DBG_MSG_SIZE);  // Receive 4 Bytes of data
	ChangeOPFrequency((float)222.29,&current_op); // set the Operating state to minimal power with PING IMPEDANCE connected 
	//ADC
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_Delay(10);
	MEAS_ADC_start(1,2); // start the ADC conversion
	//DAC
	//mereni
	unsigned int meas_increment = 0;
	float sum_current = 0.;
	//HAL_COMP_Start_IT(&hcomp2); ////HAZELO TO FURT ERRORY
	//timer pro analog ping
	HAL_TIM_Base_Init(&htim6);
	
	//TEST BEZ UARTU
	StartInverter(&current_op);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	//KONEC TESTU
	//ASK modulace
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	extern bool ASK_error;
	extern bool ASK_byte_done;
	DecodedDataPacket ASK_packet;	
	//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET); //debug
	char DBG_UART_buffer[DBG_UART_MSG_SIZE];
	//char DBG_UART_buffer2[30];
	//debug
	uint8_t test_byte_ASK;
	bool ASK_msg_done=false;
	//pid
	bool pid_enable;
	uint8_t pid_iteration=0;
	int8_t s_control_error; //signed value
	float td_j; //calculated new current throufh primary cell
	float e_ji; //error of regulation
	float e_ji_prev;
	float i_sum;
	float i_sum_prev;
	uint8_t error_code=0;
	//float pid;
	float new_freq;
	float e_i[PID_ITERATIONS];
	extern bool preamble_done;
	extern uint8_t preamble_ones;
	uint8_t byte_counter=0;
	uint16_t bytes[3]={0,0,0};

	bool first_half_FSK=true;
	bool* FSK_byte;
	uint8_t FSK_index = 0;
	bool ce_timeout=false;
	uint8_t current_state=INITIAL_STATE;
	uint16_t ping_timeout=0;
	uint8_t test_FSK_byte=0;
	Queue FSK_Queue;
	DataPacketFSK data_packet_FSK;
	data_packet_FSK.message_length=1;
	queue_init(&FSK_Queue);
	uint16_t ack_timeout = 0;
	bool expecting_ack=false;
	bool user_data_input=false;
	bool print_ce=false;
	bool no_mute=true;
	bool repeating_msg=false;
	//unsigned char test_byte=0x01;
	//test
	//ASK_byte= 
	//error_type=demodulateAskByte(ASK_byte,ASK_packet,ASK_msg_position);
				
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (FSK_interrupt){
			//ModulateFrequency(&current_op);
			if (first_half_FSK){
				ModulateFrequency(&current_op);
			}else{
				if (serialized_data_FSK[FSK_index]){
					ModulateFrequency(&current_op);
				}				
				if (FSK_index>=FSK_size){
					FSK_index=0;
					new_FSK=false;
				} else{
					FSK_index++;
				}
			}
			first_half_FSK=!first_half_FSK;
			FSK_interrupt=false;
		}
		if (analog_ping_done){
				analog_ping_done=false;
				//HAL_Delay(1);
				if (act_vals.current>0.1){ // change the value
					//receiver_detected
					current_state=DIGITAL_PING;
					ChangeOPFrequency((float)PING_IMP_FREQUENCY,&current_op); 
					ping_timeout=0;
					HAL_Delay(50);
					sprintf(DBG_UART_buffer,"R detected\r\n"); 
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
					StartInverter(&current_op);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
					number_of_ms=0;
				} else {
					ping_timeout=0;
					sprintf(DBG_UART_buffer,"R not detected.\r\n"); 
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
				
				}
		}
		if (comp_treshold){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			comp_treshold=false;
			error_code=0x05;
			
			current_state=ANALOG_PING;
			HAL_Delay(20);
			sprintf(DBG_UART_buffer,"Overcurrent! Switching off!\r\n");
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
			HAL_Delay(30);
				
		}
		if (ASK_byte_done){
			ASK_byte_done=false;		
			error_type=demodulateAskByte(ASK_byte_main,&ASK_packet,ASK_msg_position);
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			if (ASK_msg_position==3){//current_ASK_msg_size){ //length of CE data packet
				ASK_msg_position=0;
				ASK_msg_done=true;
			}
			if (error_type==0){ //no error
				/*if (ASK_msg_position==1){
					current_ASK_msg_size=1+ASK_packet.message_length+1; 
				}*/
			} else{
				ASK_error=true;
			}					
			if (ASK_msg_done){
				if (error_type!=0){
					if (ASK_packet.header!=0x03){
						sprintf(DBG_UART_buffer,"H:%u\r\nM:%u\r\nEM:%d\r\n",ASK_packet.header,ASK_packet.message[0],error_type);	
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
						}
					//HAL_Delay(50);
				} else{
					//sprintf(DBG_UART_buffer,"H:%u\r\nM:%u\r\n",ASK_packet.header,ASK_packet.message[0]);
					//HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
					//HAL_Delay(50);
				}
				//PID
				if (ASK_packet.header==0x04){ //RP packet
					if (!queue_is_empty(&FSK_Queue)){
						serialize_response_pattern(0b11001100,serialized_data_FSK); //ATN
						FSK_size=8;
						sprintf(DBG_UART_buffer,"sending ATN\r\n");	
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
						HAL_Delay(1);
						new_FSK=true;
					} else {
						serialize_response_pattern(0b11111111,serialized_data_FSK); //ACK
						FSK_size=8;
						if (no_mute){
						sprintf(DBG_UART_buffer,"RP received. Sending ACK\r\n");	
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
						}
						HAL_Delay(1);
						new_FSK=true;
												
					}
				} else if (ASK_packet.header==0x15){ //incomming data
						if (ASK_packet.message[0]==0x33){
							if (queue_dequeue(&FSK_Queue,&data_packet_FSK)){						
								serialize_data_packet(data_packet_FSK,serialized_data_FSK);
								FSK_size=get_data_packet_size(data_packet_FSK);
								sprintf(DBG_UART_buffer,"DSR: poll -> sending user data %u\r\n", test_FSK_byte);	
								HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
								HAL_Delay(1);
								ack_timeout=0;
								expecting_ack=true;
								new_FSK=true;
							}
						} else if (ASK_packet.message[0]==0xFF){
								sprintf(DBG_UART_buffer,"DSR: ack -> Receiver received the data.\r\n");	
								HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
								ack_timeout=0;
								expecting_ack=false;
								repeating_msg=false;
								HAL_Delay(1);	
						} else if (ASK_packet.message[0]==0x00){
								sprintf(DBG_UART_buffer,"DSR: nak -> repeating msg.\r\n");
								if (repeating_msg){
								sprintf(DBG_UART_buffer,"DSR: nak -> Receiver not acknowledged. User data enqued.\r\n");	
								ack_timeout=0;
								if (queue_enqueue(&FSK_Queue, data_packet_FSK)){
								} else{
								sprintf(DBG_UART_buffer,"QUEUE FULL\r\n");
								}
								repeating_msg=true;
								}
								HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
								HAL_Delay(1);		
						}
						else {
								sprintf(DBG_UART_buffer,"DSR: msg not recognized.\r\n");	
								HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
								HAL_Delay(1);
						}
				}
				else if (ASK_packet.header==0x03){
					if (print_ce){
						sprintf(DBG_UART_buffer,"CE: %u.\r\n",ASK_packet.message[0]);	
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
					}
					if (ASK_packet.message[0]!=0){ //if CE is not 0 do the regulation process
						if (pid_enable){
							e_ji_prev=0;
							i_sum_prev=0;
							//Now check if MSB is set to 1 which means that the value is negative
							if (ASK_packet.message[0] & 0x80) { // check if MSB is set
							  s_control_error = (int8_t)ASK_packet.message[0]| 0xFFFFFF00; // set higher-order bits to 1  
							} else {
							  s_control_error = (int8_t)ASK_packet.message[0]; // sign extend is not needed if MSB is not set
							}
							pid_iteration=PID_ITERATIONS;
							
						}
					}
					number_of_ms=0;
				} else if(ASK_packet.header==0x02){ //EPT
					if (ASK_packet.message[0]==0x01){//cc charge complete
						current_state=COMM_ONLY_STATE;
						sprintf(DBG_UART_buffer,"EPT/cc received. PT>CS\r\n");
						pid_enable=false;
						ChangeOPFrequency((float)PING_IMP_FREQUENCY,&current_op);
					}else if(ASK_packet.message[0]==0x0C){
						sprintf(DBG_UART_buffer,"EPT/rep. Switching to AP.\r\n");
						current_state=ANALOG_PING;
						ping_timeout=0;
						pid_enable=false;
					}
					else{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
						sprintf(DBG_UART_buffer,"EPT received! Switching off!\r\n");
						current_state=INITIAL_STATE; //should be fault state
					}
					
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
					HAL_Delay(20);
				} else if(ASK_packet.header==0x01){ //SIG
						current_state=COMM_ONLY_STATE;
						HAL_Delay(10);
						sprintf(DBG_UART_buffer,"Comm. ready\r\n");
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	

					//HAL_Delay(20);
					//todo:osetreni timeoutu
					//todo:zpracovani hodnoty
				}else if(ASK_packet.header==0x20){ //SRQ/en
					if (current_state!=FAULT_STATE){
						sprintf(DBG_UART_buffer,"SRQ/en received. Sending ACK. CS->PT.\r\n");
						if (current_state!=POWER_TRANSFER_STATE){
							pid_enable=true;
							current_state=POWER_TRANSFER_STATE;
							serialize_response_pattern(0b11111111,serialized_data_FSK); //ACK
							FSK_size=8;
							HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
							HAL_Delay(1);
							new_FSK=true;
							}
					} else{
						sprintf(DBG_UART_buffer,"SRQ/en received. RESOLVE the fault first.\r\n");
						HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
					}
					number_of_ms=0;
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
				} else if(ASK_packet.header==0x18){ //PROP... USER DATA EXAMPLE
					sprintf(DBG_UART_buffer,"User data from Receiver: %c Sending ACK.\r\n",ASK_packet.message[0]);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
					serialize_response_pattern(0b11111111,serialized_data_FSK); //ACK
					FSK_size=8;
					HAL_Delay(1);
					new_FSK=true;				
				}
				else{
					sprintf(DBG_UART_buffer,"%u header\r\n",ASK_packet.header);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));	
					HAL_Delay(20);
				}
				ASK_msg_done=false;			
			}
		}
		if (ASK_error){
				preamble_done=false;
				preamble_ones=0;
				ASK_error=false;
				ASK_msg_position=0;
				ASK_packet.header=0;
				ASK_packet.message[0]=0;
				//sprintf(DBG_UART_buffer,"E:%d\r\n",error_type);				
				//HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
				error_type=0;
		}
		/* Has 1 ms passed since the last time? */
		if (one_ms){
			if (expecting_ack){
				ack_timeout++;
				if (ack_timeout>1000){
				ack_timeout=0;
				if (!repeating_msg){
				repeating_msg=true;
				ack_timeout=0;
				if (queue_enqueue(&FSK_Queue, data_packet_FSK)){
					sprintf(DBG_UART_buffer,"Timeout-Receiver not acknowledged. User data enqued.\r\n");	
				} else{
					sprintf(DBG_UART_buffer,"QUEUE FULL\r\n");
					}
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
				}
				}
			}
			if (current_state==DIGITAL_PING){
				ping_timeout++;
				if (ping_timeout==300){
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
				} else if (ping_timeout>=1500){
					current_state=ANALOG_PING;
					ping_timeout=0;
				}
			} else if (current_state==ANALOG_PING){
				ping_timeout++;
				if (ping_timeout>=500){
					AnalogPing(&current_op);
					analog_ping_measure=true;
					ping_timeout=0;
				}
			}
			if (number_of_ms%200){
				if (current_state==FAULT_STATE){
					sprintf(DBG_UART_buffer,"Error: %u. Resolve the error with 'e'.\r\n",error_code);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
				}	
			}
			if ((number_of_ms%4)&&(number_of_ms>T_DELAY)){
				if (pid_iteration!=0&&pid_enable){
					MEAS_ADC_start(20,0);
					while (!MEAS_ACD1_update){}
					MEAS_ACD1_update=0;
					if (pid_iteration==PID_ITERATIONS){
						//i_pred=act_vals.current;
						td_j=act_vals.current*(1.+((float)s_control_error)/128.);
						//sprintf(DBG_UART_buffer,"CE received\r\n");	
						//HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
						//ta_ji_prev=act_vals.current;
					}
					e_ji=td_j-act_vals.current;
					i_sum=i_sum_prev+K_i*e_ji*0.004;//4 ms
					new_freq=current_op.op_frequency-(K_p*e_ji+i_sum+K_d*(e_ji-e_ji_prev)/0.004);
					i_sum_prev=i_sum;
					e_ji_prev=e_ji;
					//sprintf(DBG_UART_buffer,"freq:%f\r\nnew_freq:%f\r\ne_ji:%f\r\n",current_op.op_frequency,new_freq,e_ji);
					//HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
					//HAL_Delay(200);
					ChangeOPFrequency(new_freq,&current_op);
					HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
					e_i[PID_ITERATIONS-pid_iteration]=e_ji;
					pid_iteration--;
					if (pid_iteration==0){
						//HAL_Delay(10);
						MEAS_ADC_start(20,0);
						//while (!MEAS_ACD1_update){}
						//MEAS_ACD1_update=0;
						//HAL_Delay(100);
						//sprintf(DBG_UART_buffer,"CE:%d\r\ne1:%f\r\ne2:%f\r\ne3:%f\r\ne4:%f\r\ne5:%f\r\nIN:%f\r\n\r\n",s_control_error,e_i[0],e_i[1],e_i[2],e_i[3],e_i[4],act_vals.current);
						//HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));		
					}
					//sprintf(DBG_UART_buffer,"e_ji:%f\r\n",e_ji);
					
				}
			}
			button = !HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
			if ((number_of_ms%MES_INTERVAL)==0&&pid_iteration==0){
				while (!MEAS_ACD1_update){}
				MEAS_ACD1_update=0;
				/*if (act_vals.current<0.05&&(current_state == COMM_ONLY_STATE)){ //puv 0.08
					error_code=1;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
					current_state=FAULT_STATE;
					HAL_Delay(20);
					sprintf(DBG_UART_buffer,"Rec not present! Switching off!\r\n");
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
					HAL_Delay(30);			
				}
				*/
				if (act_vals.current>1.35){
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
					comp_treshold=false;
					error_code=0x06;
					current_state=INITIAL_STATE;
					
					modulateFskByte(data_packet_FSK.header, 0x16); //ADT 1 byte
					modulateFskByte(data_packet_FSK.message[0],1); //test value
					modulateFskByte(data_packet_FSK.checksum,calculateCheckSum(data_packet_FSK));
					if (queue_enqueue(&FSK_Queue, data_packet_FSK)){
						sprintf(DBG_UART_buffer,"\r\nUser data enqued: %c\r\n",Rx_data[0]);
					} else{
						sprintf(DBG_UART_buffer,"QUEUE FULL\r\n");
					}
					
					HAL_Delay(20);
					sprintf(DBG_UART_buffer,"Overcurrent! Switching off!\r\n");
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
					HAL_Delay(30);
				}
				MEAS_ADC_start(10,1);
			}
			if (number_of_ms%1000){
				/*if (run_test){
					sprintf(DBG_UART_buffer,"I:%fA\r\nU:%fV\r\nP:%fW\r\n\r\n",act_vals.current,act_vals.voltage,act_vals.power);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
				}*/
			}
			if (number_of_ms>=3000){
				number_of_ms=0;
				if (run_test){ //normalne nikdy neprobehne !!!
					sprintf(DBG_UART_buffer,"I:%fA\r\nU:%fV\r\nP:%fW\r\n\r\n",act_vals.current,act_vals.voltage,act_vals.power);
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
				}
				if (pid_enable&&current_state==POWER_TRANSFER_STATE){
					//ce_timeout dodelat
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
					ChangeOPFrequency((float)PING_IMP_FREQUENCY,&current_op); //back to ping impedance
					sprintf(DBG_UART_buffer,"No CE packet. Switching off.\r\n\r\n");
					HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
					HAL_Delay(20);
					ce_timeout=true;
					pid_enable=false;
					current_state=ANALOG_PING;
					HAL_Delay(500);
					AnalogPing(&current_op);
					analog_ping_measure=true;
					//current_state=INITIAL_STATE;
				}
			}
			//zmena na EUROC -> Transmitter se bude pokoušet porád o spojení
			if (current_state==INITIAL_STATE){
						sprintf(DBG_UART_buffer,"AP initiated.\r\n");
						current_state=ANALOG_PING;
						AnalogPing(&current_op);
						analog_ping_measure=true;
						}
			one_ms=false;
			number_of_ms++;
			
			//processDBGUART();
		if (new_dbg_uart_msg){
			new_dbg_uart_msg=false;
				if (user_data_input){
					//test_FSK_byte++;
					modulateFskByte(data_packet_FSK.header, 0x16); //ADT 1 byte
					modulateFskByte(data_packet_FSK.message[0],Rx_data[0]); //test value
					modulateFskByte(data_packet_FSK.checksum,calculateCheckSum(data_packet_FSK));
					if (queue_enqueue(&FSK_Queue, data_packet_FSK)){
						sprintf(DBG_UART_buffer,"\r\nUser data enqued: %c\r\n",Rx_data[0]);
					} else{
						sprintf(DBG_UART_buffer,"QUEUE FULL\r\n");
					}
						user_data_input=false;
				} else{
			switch (Rx_data[0])
			{
				case 'c':
					sprintf(DBG_UART_buffer,"Incomming CE packets will be printed.\r\n");
					print_ce=true;
					break;
				case 'C':
					sprintf(DBG_UART_buffer,"Incomming CE packets will not be printed.\r\n");
					print_ce=false;
					break;						
				case '+':
					ChangeOPFrequency(current_op.op_frequency+0.01,&current_op);
					sprintf(DBG_UART_buffer,"Frequency set to: %f\r\n\r\n",current_op.op_frequency);
					break;
				case '-':
					ChangeOPFrequency(current_op.op_frequency-0.01,&current_op);
					sprintf(DBG_UART_buffer,"Frequency set to: %f\r\n\r\n",current_op.op_frequency);
					break;
				case '.':
					ChangeOPFrequency(current_op.op_frequency+0.5,&current_op);
					sprintf(DBG_UART_buffer,"Frequency set to: %f\r\n\r\n",current_op.op_frequency);
					break;
				case ',':
					ChangeOPFrequency(current_op.op_frequency-0.5,&current_op);
					sprintf(DBG_UART_buffer,"Frequency set to: %f\r\n\r\n",current_op.op_frequency);
					break;
				case 'r':
					StartInverter(&current_op);
					sprintf(DBG_UART_buffer,"Switching signals restored!\r\n\r\n");
					break;
				case 's':
					StopInverter(&current_op);
					sprintf(DBG_UART_buffer,"Switching signals stopped!\r\n\r\n");
					break;
				case 'l':
					HAL_GPIO_TogglePin(GPIOB,LED3);
					sprintf(DBG_UART_buffer,"LED3 Toggled!\r\n\r\n");
					break;
				case '0':
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
					current_state=INITIAL_STATE;
					sprintf(DBG_UART_buffer,"Drivers output disabled!\r\n\r\n");
					break;
				case '1':
					if (current_op.gate_signals){
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
						sprintf(DBG_UART_buffer,"Drivers output enabled!\r\n\r\n");
						}
					else{
						sprintf(DBG_UART_buffer,"No switching signals to gate!\r\n\r\n");
						}
					break;
				case 'e':
						sprintf(DBG_UART_buffer,"Fault resolved.\r\n");
						current_state=INITIAL_STATE;
						error_code=0;
						break;
				case 'a':
					if (current_state==INITIAL_STATE){
						sprintf(DBG_UART_buffer,"AP initiated.\r\n");
						current_state=ANALOG_PING;
						AnalogPing(&current_op);
						analog_ping_measure=true;
					}else{
						sprintf(DBG_UART_buffer,"AP ended.\r\n");
						current_state=INITIAL_STATE;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
						HAL_Delay(20);
					}
					break;
				case 'm':
					no_mute=!no_mute;
					sprintf(DBG_UART_buffer,"MSGs (un)muted!\r\n");
					break;
					
				case 'v':
					//MEAS_ADC_start(50,1);
					//while (!MEAS_ACD1_update){}
					//MEAS_ACD1_update=0;
					sprintf(DBG_UART_buffer,"current: %f A\r\nVoltage: %f V\r\n\r\n",act_vals.current,act_vals.voltage);
					break;
				case 't':
					run_test=!run_test;
					MEAS_ADC_start(10,1);
					if (run_test)sprintf(DBG_UART_buffer,"Test started. Sending data:\r\n\r\n");
					else sprintf(DBG_UART_buffer,"End of test.\r\n\r\n");
					break;
				case 'P':
					sprintf(DBG_UART_buffer,"PID enabled\r\n");
					pid_enable=true;
					break;
				case 'p':
					sprintf(DBG_UART_buffer,"PID disabled\r\n");
					pid_enable=false;
					break;
				case 'K':
					K_p+=1;
					sprintf(DBG_UART_buffer,"Kp:%f\r\n",K_p);
					break;
				case 'k':
					K_p-=1;
					sprintf(DBG_UART_buffer,"Kp:%f\r\n",K_p);
					break;
				case 'I':
					K_i+=1;
					sprintf(DBG_UART_buffer,"Ki:%f\r\n",K_i);
					break;
				case 'i':
					K_i-=1;
					sprintf(DBG_UART_buffer,"Ki:%f\r\n",K_i);
					break;
				case 'D':
					K_d+=0.1;
					sprintf(DBG_UART_buffer,"Kd:%f\r\n",K_d);
					break;
				case 'd':
					K_d-=0.1;
					sprintf(DBG_UART_buffer,"Kd:%f\r\n",K_d);
					break;
				case 'F':
					//FSK_byte = modulateFskByte(test_FSK_byte);
					test_FSK_byte++;
					new_FSK=true;
					sprintf(DBG_UART_buffer,"FSK_true");
					break;
				case 'f':
					new_FSK=false;
					sprintf(DBG_UART_buffer,"FSK_false");
					break;
				case 'q': //ATN nejdriv ale - user data
					sprintf(DBG_UART_buffer,"ENTER user data: ");
					user_data_input=true;
					break;
				default:
					sprintf(DBG_UART_buffer,"Invalid char!\r\n\r\n");
					break;
					
			}
			}
			HAL_UART_Transmit_IT(&huart1, (uint8_t *)DBG_UART_buffer, strlen(DBG_UART_buffer));
		}
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ChangeOPFrequency(float frequency,op_values *current_op){ //in kHz
	if (frequency>MAX_FREQUENCY){
		frequency=MAX_FREQUENCY;
	}
	if (frequency<MIN_FREQUENCY){
		frequency=MIN_FREQUENCY;
	}
	float aux_var;
	current_op->op_frequency=frequency;
	current_op->op_period=1/current_op->op_frequency;
	aux_var=current_op->op_period*1000000000;
	aux_var=aux_var/HRTIM_RESOLUTION;
	current_op->op_period_in_ticks= (uint32_t) aux_var;
	current_op->mod_period_in_ticks= current_op->op_period_in_ticks-TICKS_DELTA_MODULATION;
	__HAL_HRTIM_SETPERIOD(&hhrtim1,0x2, current_op->op_period_in_ticks); //set timer C period
}


void ModulateFrequency(op_values *current_op){
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
	static bool iteration = true;
	if (iteration){
	__HAL_HRTIM_SETPERIOD(&hhrtim1,0x2, current_op->mod_period_in_ticks); 
	}else{
	__HAL_HRTIM_SETPERIOD(&hhrtim1,0x2, current_op->op_period_in_ticks);
	}
	iteration=!iteration;
}


void StopInverter(op_values *current_op){
	HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
	HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_C);
	HAL_HRTIM_WaveformCountStop_IT(&hhrtim1, HRTIM_TIMERID_TIMER_C);
	current_op->gate_signals=false;
	//HAL_HRTIM_WaveformSetOutputLevel
	//pozn. po vypnutí dojde k tomu, že výstupy zustanou v neaktivním stavu - pro output 1 to je LOW, pro output 2 to je HIGH
	// asi by stacilo dát SD na driveru do off - na LO i HI strane driveru bude 0.
}
void StartInverter(op_values *current_op){
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1|HRTIM_OUTPUT_TC2);
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_C);
	current_op->gate_signals=true;
}

void StartDemodulator(void){
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_A);
}

void AnalogPing(op_values *current_op){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	StopInverter(current_op);
	ChangeOPFrequency(ANALOG_PING_FREQUENCY,current_op);
	StartInverter(current_op);
	HAL_Delay(10);
	analog_ping_counter = 0;
	analog_ping_done=false;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim6);
	}
	
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
   if(hadc) MEAS_ADC1_eval(1);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
   if(hadc) MEAS_ADC1_eval(0);
}

/* USER CODE END 4 */

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
