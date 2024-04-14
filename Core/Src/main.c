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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PWM_UPPER_LIMIT 65535
#define PWM_LOWER_LIMIT 10000
#define TICKS_PER_REVOLUTION 1120
#define DISTANCE_THRESHOLD 10
#define MAX_BUFFER_SIZE 25
#define R  0.05
#define LX 0.094
#define LY 0.126
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//USEFUL FUNCTIONS
char msg[400];
double map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (double) (x - in_min) * (out_max - out_min) / (in_max - in_min)
			+ out_min;
}
int32_t percent2pwm(int x) {
	return (x - 0) * (65535 - 0) / (100 - 0) + 0;
}
int32_t rpm2pwm(int x) {
	return (x - 0) * (65535 - 0) / (160 - 0) + 0;
}
double rad2deg(double rad) {
	return (rad * (180.0 / M_PI)) - ((int) (rad * (180.0 / M_PI) / 360) * 360);
}
float rad2rpm(float rad) {
    return roundf((rad * 60) / (2 * M_PI));
}
//MOTOR STRUCT
typedef struct{
	int32_t position, old_position;
	int speed;
	float speed_rpm, speed_rad;
}MOTOR;

void MOTOR_Init(MOTOR *motor){
	motor->position = 0;
	motor->old_position = 0;
	motor->speed = 0;
	motor->speed_rpm = 0;
	motor->speed_rad = 0;
}

//CONTROLLER STRUCT
typedef struct{
	int w, y, x;
}Controller;

void Controller_Init(Controller *ps4){
	ps4->w = 0;
	ps4->y = 0;
	ps4->x = 0;
}

//COORDINATES STRUCT
/*TODO: Coordinates struct if needed?*/
/*
typedef struct{
	double current_x, current_y, current_h;
} ODO;

void ODO_Init(ODO *odometer){
	odometer->current_x = 0;
	odometer->current_y = 0;
	odometer->current_h = 0;
}
*/

//LIVE FOLLOWER BUFFER
uint8_t live_follower_buffer[25];

//TELEOP VARIABLES
Controller ps4;
uint8_t controller_buffer[15];

//ODOMETRY VARIABLES
uint8_t odometry_buffer[25];
char tx_buffer[MAX_BUFFER_SIZE];
int chars_written = 0;
double current_x = 0, current_y = 0, current_h = 0;


void process_data() {
	chars_written = snprintf(tx_buffer, MAX_BUFFER_SIZE, "%+.2f,%+.2f,%+.2f\n",
			current_x, current_y, current_h);
	if (chars_written < 0 || chars_written > MAX_BUFFER_SIZE) {
		/*TODO: handle error*/
	} else {
		for (int i = chars_written; i < MAX_BUFFER_SIZE; i++) {
			tx_buffer[i] = '\n';
		}
	}
	tx_buffer[MAX_BUFFER_SIZE - 1] = '\0';
}

int small_counter = 0, big_counter = 0;
MOTOR fdr, fst, sdr, sst;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim9) {
		//fdr
		fdr.position = __HAL_TIM_GET_COUNTER(&htim1);
		fdr.speed = fdr.position - fdr.old_position;
		fdr.speed_rpm = (fdr.speed * 60) / TICKS_PER_REVOLUTION;
		fdr.speed_rad = (fdr.speed_rpm / 60) * 2 * M_PI;
		fdr.old_position = fdr.position;

		//fst
		fst.position = __HAL_TIM_GET_COUNTER(&htim1);
		fst.speed = fst.position - fst.old_position;
		fst.speed_rpm = (fst.speed * 60) / TICKS_PER_REVOLUTION;
		fst.speed_rad = (fst.speed_rpm / 60) * 2 * M_PI;
		fst.old_position = fst.position;

		//sdr
		sdr.position = __HAL_TIM_GET_COUNTER(&htim1);
		sdr.speed = sdr.position - sdr.old_position;
		sdr.speed_rpm = (sdr.speed * 60) / TICKS_PER_REVOLUTION;
		sdr.speed_rad = (sdr.speed_rpm / 60) * 2 * M_PI;
		sdr.old_position = sdr.position;

		//sst
		sst.position = __HAL_TIM_GET_COUNTER(&htim1);
		sst.speed = sst.position - sst.old_position;
		sst.speed_rpm = (sst.speed * 60) / TICKS_PER_REVOLUTION;
		sst.speed_rad = (sst.speed_rpm / 60) * 2 * M_PI;
		sst.old_position = sst.position;
		small_counter++;
	}
	if(htim == &htim12){
//		process_data();
//		HAL_UART_Transmit(&huart2, (uint8_t *) tx_buffer, 25, HAL_MAX_DELAY);
//		big_counter++;
	}
}


int delay = 1000;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart2){
//		process_data();
//		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) tx_buffer,25);
//		big_counter++;
//		HAL_Delay(delay);
//	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) { //ESP32
		HAL_UART_Receive_DMA(&huart2, controller_buffer, 15);
		int joystick_values[3];
		int index = 0;
		char *token;

		token = strtok((char*) controller_buffer, ",");
		while (token != NULL && index < 3) {
			joystick_values[index++] = atoi(token);
			token = strtok(NULL, ",");
		}
		ps4.w = joystick_values[0]; //lx - w
		ps4.y = joystick_values[1]; //rx - y
		ps4.x = joystick_values[2]; //ry - x
	}
	if (huart == &huart6) { //BLUEPILL
		HAL_UART_Receive_DMA(&huart6, odometry_buffer, 25);
		double coordinates[3];
		int index = 0;
		char *token;

		token = strtok((char*) odometry_buffer, ",");
		while (token != NULL && index < 3) {
			coordinates[index++] = strtod(token, NULL);
			token = strtok(NULL, ",");
		}
		current_x = (double) ((int) (coordinates[0] * 100)) / 100;
		current_y = (double) ((int) (coordinates[1] * 100)) / 100;
		current_h = (double) ((int) (coordinates[2] * 100)) / 100;
//		process_data();
//		HAL_UART_Transmit(&huart2, (uint8_t*) tx_buffer,25, HAL_MAX_DELAY);
	}
}

//MOTORS MOVEMENT
void fdr_set_pwm(int32_t pwm_value) {
	if (pwm_value > 0) {
		TIM4->CCR3 = 0; // L
		TIM4->CCR4 = pwm_value; // R
	} else if (pwm_value < 0) {
		TIM4->CCR4 = 0; // R
		TIM4->CCR3 = pwm_value * -1; // L
	} else if (pwm_value == 0) {
		TIM4->CCR4 = 0; // R
		TIM4->CCR3 = 0; // L
	}
}
void fst_set_pwm(int32_t pwm_value) {
	if (pwm_value > 0) {
		TIM4->CCR2 = 0; // L
		TIM4->CCR1 = pwm_value; // R
	} else if (pwm_value < 0) {
		TIM4->CCR1 = 0; // R
		TIM4->CCR2 = pwm_value * -1; // L
	} else if (pwm_value == 0) {
		TIM4->CCR1 = 0; // R
		TIM4->CCR2 = 0; // L
	}
}
void sdr_set_pwm(int32_t pwm_value) {
	if (pwm_value > 0) {
		TIM8->CCR3 = 0; // R
		TIM8->CCR4 = pwm_value; // L
	} else if (pwm_value < 0) {
		TIM8->CCR4 = 0; // L
		TIM8->CCR3 = pwm_value * -1; // R
	} else if (pwm_value == 0) {
		TIM8->CCR4 = 0; // R
		TIM8->CCR3 = 0; // L
	}
}
void sst_set_pwm(int32_t pwm_value) {
	if (pwm_value > 0) {
		TIM8->CCR2 = 0; // L
		TIM8->CCR1 = pwm_value; // R
	} else if (pwm_value < 0) {
		TIM8->CCR1 = 0; // R
		TIM8->CCR2 = pwm_value * -1; // L
	} else if (pwm_value == 0) {
		TIM8->CCR1 = 0; // R
		TIM8->CCR2 = 0; // L
	}
}
void run_motors(double fdr_rpm, double sdr_rpm, double fst_rpm, double sst_rpm) {
	fdr_set_pwm(rpm2pwm(fdr_rpm));
	sdr_set_pwm(rpm2pwm(sdr_rpm));
	fst_set_pwm(rpm2pwm(fst_rpm));
	sst_set_pwm(rpm2pwm(sst_rpm));
}
void stop_motors() {
	fdr_set_pwm(0);
	sdr_set_pwm(0);
	fst_set_pwm(0);
	sst_set_pwm(0);
}
/*TODO: Implement wait for start
 * Do i even need wait for start?*/
void wait_for_start() {
	while (1) {
		/*TODO: Power a led to know that you are in waiting mode*/
		/*TODO: Implement beaking condition*/
//		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7)) {
//			fdr(0);
//			sdr(0);
//			fst(0);
//			sst(0);
//			break;
//		}
	}
}

void tele_op_mode() {
	int32_t fdr_pwm = 0, sdr_pwm = 0, fst_pwm = 0, sst_pwm = 0;
	double fdr_ref = 0, sdr_ref = 0, fst_ref = 0, sst_ref = 0;
	HAL_UART_Receive_DMA(&huart2, controller_buffer, 15);
	//lx - ps4.w
	//rx - ps4.y
	//ry - ps4.x
	while (1) {
		HAL_UART_Receive_DMA(&huart2, controller_buffer, 15);
		fdr_ref = map(-ps4.x, -512, 512, -1, 1) + map(-ps4.y, -512, 512, -1, 1)
				+ map(-ps4.w, -512, 512, -1, 1);
		sdr_ref = map(-ps4.x, -512, 512, -1, 1) - map(-ps4.y, -512, 512, -1, 1)
				+ map(-ps4.w, -512, 512, -1, 1);
		fst_ref = map(-ps4.x, -512, 512, -1, 1) - map(-ps4.y, -512, 512, -1, 1)
				- map(-ps4.w, -512, 512, -1, 1);
		sst_ref = map(-ps4.x, -512, 512, -1, 1) + map(-ps4.y, -512, 512, -1, 1)
				- map(-ps4.w, -512, 512, -1, 1);

		fdr_pwm = percent2pwm(fdr_ref * 100);
		sdr_pwm = percent2pwm(sdr_ref * 100);
		fst_pwm = percent2pwm(fst_ref * 100);
		sst_pwm = percent2pwm(sst_ref * 100);

		fdr_pwm =
				(fdr_pwm > PWM_UPPER_LIMIT) ? PWM_UPPER_LIMIT :
				(fdr_pwm < -PWM_UPPER_LIMIT) ? -PWM_UPPER_LIMIT :
				(fdr_pwm > -PWM_LOWER_LIMIT && fdr_pwm < PWM_LOWER_LIMIT) ?
						0 : fdr_pwm;
		sdr_pwm =
				(sdr_pwm > PWM_UPPER_LIMIT) ? PWM_UPPER_LIMIT :
				(sdr_pwm < -PWM_UPPER_LIMIT) ? -PWM_UPPER_LIMIT :
				(sdr_pwm > -PWM_LOWER_LIMIT && sdr_pwm < PWM_LOWER_LIMIT) ?
						0 : sdr_pwm;
		fst_pwm =
				(fst_pwm > PWM_UPPER_LIMIT) ? PWM_UPPER_LIMIT :
				(fst_pwm < -PWM_UPPER_LIMIT) ? -PWM_UPPER_LIMIT :
				(fst_pwm > -PWM_LOWER_LIMIT && fst_pwm < PWM_LOWER_LIMIT) ?
						0 : fst_pwm;
		sst_pwm =
				(sst_pwm > PWM_UPPER_LIMIT) ? PWM_UPPER_LIMIT :
				(sst_pwm < -PWM_UPPER_LIMIT) ? -PWM_UPPER_LIMIT :
				(sst_pwm > -PWM_LOWER_LIMIT && sst_pwm < PWM_LOWER_LIMIT) ?
						0 : sst_pwm;

		fdr_set_pwm(fdr_pwm);
		sdr_set_pwm(sdr_pwm);
		fst_set_pwm(fst_pwm);
		sst_set_pwm(sst_pwm);

		/*TODO: Implement beaking condition*/
//		if(conditie){
//			break;
//		}
	}
}
//double points[3][2] = { { 1.6, 0.0 }, { 1.6, -1.1 }, { 0.0, -1.1 } };
//double speeds[3][4] = { { 35, 35, 35, 35 }, { 35, -35, -35, 35 }, { -35, -35, -35, -35 } };
double final_x = 0, final_y = 0;
double points[2][2] = { { 100, 0.0 },
						{ 100, 100.0 } };

double w_fdr = 0, w_fst = 0, w_sdr = 0, w_sst = 0;
double max_speed = 0, scale_factor = 0;
void calculate_speed(double Vx, double Vy, double Wz){
	//pentru directia rotilor ne luam dupa asta https://www.itm-conferences.org/articles/itmconf/pdf/2020/05/itmconf_itee2020_04001.pdf
	w_fst = ((1 / R) * (Vx - Vy - ((LX + LY) * Wz))) / 40;
	w_fdr = ((1 / R) * (Vx + Vy + ((LX + LY) * Wz))) / 40;
	w_sst = ((1 / R) * (Vx + Vy - ((LX + LY) * Wz))) / 40;
	w_sdr = ((1 / R) * (Vx - Vy + ((LX + LY) * Wz))) / 40;

	max_speed = fmax(fmax(fabs(w_fst), fabs(w_fdr)), fmax(fabs(w_sst), fabs(w_sdr)));
	if( max_speed > 12.5){
		scale_factor = fabs(12.5 / max_speed);
		w_fst *= scale_factor;
		w_fdr *= scale_factor;
		w_sst *= scale_factor;
		w_sdr *= scale_factor;
	}else if(max_speed < 8.0){
		scale_factor = fabs(8.0 / max_speed);
		w_fst *= scale_factor;
		w_fdr *= scale_factor;
		w_sst *= scale_factor;
		w_sdr *= scale_factor;

	}

	w_fst = rad2rpm(w_fst);
	w_fdr = rad2rpm(w_fdr);
	w_sst = rad2rpm(w_sst);
	w_sdr = rad2rpm(w_sdr);


}
/*TODO: Movement error corection*/
/*TODO: Fa ceva cu asta fratelo*/
double Vx = 0, Vy = 0;
void follow_trajectory(double final_x, double final_y) {
	HAL_UART_Receive_DMA(&huart6, odometry_buffer, 25);
	if (current_x < final_x) {
		Vx = final_x - current_x;
		Vy = final_y - current_y;

		calculate_speed(Vx, Vy, 0);
		run_motors(w_fdr, w_sdr, w_fst, w_sst);
	}
	if(current_x > final_x){
		stop_motors();
	}
//	stop_motors();
}
/*TODO: Heading locking*/
//double Kp = 0.2;
//int32_t output = 0;
//double error = 0;
//double ref = 90;
//int err_threshold = 5;
//void lock_heading(double ref){
//	error = abs(ref) - abs(current_h);
//	if(ref < 0){
//		output = map(Kp * error,-36,0,-65535,15000);
//		if(output < -65535) output = -65535;
//		if(error < err_threshold && error > -err_threshold) output = 0;
//		rotate_right(output);
//	}else if(ref > 0){
//		output = map(Kp * error,0,36,15000,65535);
//		if(output > 65535) output = 65535;
//		if(error < err_threshold && error > -err_threshold) output = 0;
//		rotate_left(output);
//	}
//
//}
/*TODO: Implement auto_op_mode*/
void auto_op_mode() {
	//variabile
	while (1) {

		/*TODO: Implement beaking condition*/
//		if(conditie){
//			break;
//		}
	}
}
int x = 0;
double las_time = 0;
void send_data(){
	las_time = HAL_GetTick();
	if(HAL_GetTick() - las_time == 500){
		process_data();
		HAL_UART_Transmit(&huart2, (uint8_t *) tx_buffer, 25, HAL_MAX_DELAY);
	}
}

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim12);

	MOTOR_Init(&fdr);
	MOTOR_Init(&fst);
	MOTOR_Init(&sdr);
	MOTOR_Init(&sst);
	Controller_Init(&ps4);

	HAL_UART_Receive_DMA(&huart6, odometry_buffer, 25);
//	process_data();
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t *) tx_buffer, 25);
//	follow_trajectory();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*TODO: Implement mode switching*/

	while (1) {
//		follow_trajectory(100, 0);
//		process_data();
//		sprintf(msg, "e alo %d \r\n", x);
//		HAL_UART_Transmit(&huart2, (uint8_t *) tx_buffer, 25, HAL_MAX_DELAY);
//		send_data();
//		x++;
//		HAL_Delay(delay);
//		lock_heading(ref);
//		tele_op_mode();
//		if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7)) {
//			//TELEOP
			tele_op_mode();
//		} else if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)) {
//			//AUTO OP MODE
//			follow_trajectory();
//		} else if (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_9)) {
//			//wait
//		}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 146;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1500;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1500;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
