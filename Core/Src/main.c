/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "inttypes.h"
#include "W25Q32/W25Q32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t hum;
	uint8_t hum0;
	uint8_t tem;
	uint8_t tem0;
	uint8_t check_sum;
} dht22;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT		2				// 2 ms
#define DHT_TIMEOUT			5
#define INPUT 				0
#define OUTPUT				1
#define DURATION			30 * 1000
#define SECTOR_USE			41
#define RETRY				5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// UART
uint8_t uart_buf[100];
uint8_t tx[1024];
uint8_t uart_buf_cnt = 0;
uint8_t uart_chr;
uint8_t uart_last_rcv = 0;

// Timer
uint32_t g_sys_tick = 0;
uint32_t delay_tick = 0;
uint32_t test_tick = 0;
uint32_t dur_tick = 0;

// LED PC13
uint32_t led_tick = 0;

// LIGHT
uint8_t led1 = 0;
uint8_t led2 = 0;
uint8_t led_state = 0;
uint8_t led_state_2 = 0;
uint8_t led_mod = 0;
uint8_t led_mod_2 = 0;
uint8_t l_done;
uint8_t l_done_2;
uint32_t lss_tick = 0;

// SR501
uint8_t sr501_state = 0;
uint8_t sr501_state_2 = 0;

// SG90
uint8_t door1 = 0;
uint8_t door2 = 0;
uint8_t d_mod = 0;					// door mode (0:manual 1:auto)
uint8_t d_mod_2 = 0;
uint8_t door_state = 0;
uint8_t door_state_2 = 0;
uint8_t d_done = 0;
uint8_t d_done_2 = 0;

//buzzer
uint8_t buz_done = 0;
uint32_t buz_tick = 0;
uint8_t buz_lvl = 0;
uint8_t buz_state = 0;

//encoder motor
uint8_t fan1 = 0;
uint8_t fan2 = 0;
uint8_t speed = 0;
uint8_t speed_2 = 0;
uint8_t f_done = 0;
uint8_t f_done_2 = 0;
int motor_speed = 0;
int motor_speed_2 = 0;
uint32_t en_cnt = 0;
uint32_t en_cnt_2 = 0;

// mq2
uint16_t adc_val = 0;

// dht22
dht22 dht;
float tem = 0;
float hum = 0;
uint32_t time_out = 0;
uint8_t dht_err = 0;

// button
uint32_t btn_tick = 0;

bool rx_spi_flg = 0;

uint8_t flash_data[100];

uint8_t test = 0;

uint8_t isDebug = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void gpio_set_mode(uint8_t mode);					// input/output: 0/1

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);		// for sr501
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//void First_Start();

void Blink_Led();
void Handle_Command();

// SR501
void SR501_getState();

// SG90
void SG90_Program();
void control_door();

// encoder motor
void motor_Program();
void en_motor();
void motor_get_speed();
uint8_t check_speed(int encoder_val);

// LED
void control_led();

// Buzzer
void warning();
void buzzer();

void getADC_value();

// dht22
void delay_us(uint32_t us);
void init_dht22();
void dht22_GetValue(dht22 *dht);

void Flash_write_info();
void Flash_get_info();

// response
void Response();					// sent data to esp32
void Send_data();

void Handle_Command();
void clear_uart_buf();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, &uart_chr, sizeof(uart_chr));
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	init_dht22();
	W25Q32_Init(&hspi2, GPIOB, GPIO_PIN_12);
//	First_Start();
	Flash_get_info();
	Response();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		Blink_Led();
		Handle_Command();
		en_motor();
		control_door();
		control_led();
		Send_data();
		/* Test case for encoder motor*/
//		if (g_sys_tick - test_tick >= 1000) {
//			motor_get_speed();
//			en_cnt_2 = __HAL_TIM_GetCounter(&htim4);
//			HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
//			test_tick = g_sys_tick;
//		}
		/*Test case for servo motor*/
//		if (g_sys_tick - test_tick >= 1000) {
//			test = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
//			sprintf((char*) tx, "SR501: %d\n", test);
//			HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
//			test_tick = g_sys_tick;
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1599;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 199;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 15;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
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
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	BUZZER_Pin | LIGHT_Pin | CS_Pin | LIGHT2_Pin | DHT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN6_Pin BTN4_Pin */
	GPIO_InitStruct.Pin = BTN6_Pin | BTN4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : SR502_2_Pin LSS_Pin */
	GPIO_InitStruct.Pin = SR502_2_Pin | LSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SR501_Pin LSS2_Pin */
	GPIO_InitStruct.Pin = SR501_Pin | LSS2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : BUZZER_Pin LIGHT_Pin CS_Pin LIGHT2_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin | LIGHT_Pin | CS_Pin | LIGHT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BTN5_Pin */
	GPIO_InitStruct.Pin = BTN5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BTN5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN1_Pin BTN2_Pin BTN3_Pin */
	GPIO_InitStruct.Pin = BTN1_Pin | BTN2_Pin | BTN3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : DHT_Pin */
	GPIO_InitStruct.Pin = DHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void First_Start() {
//	l_done = 1;
//	l_done_2 = 1;
//	d_done = 1;
//	d_done_2 = 1;
//	f_done = 1;
//	f_done_2 = 1;
//}

void gpio_set_mode(uint8_t mode) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (mode) {						// output
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	} else {							// input
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

void Response() {
	dht22_GetValue(&dht);
	getADC_value();
	motor_get_speed();
//	sprintf((char*) tx, "tem:%.1f\thum:%.1f\tmq2:%d\tdr1:%" PRIu8 "\tdm1:%" PRIu8 "\tds1:%" PRIu8 "\tdr2:%"
//            PRIu8 "\tdm2:%" PRIu8 "\tds2:%" PRIu8 "\tfn1:%" PRIu8 "\tfs1:%" PRIu8 "\tfn2:%" PRIu8
//            "\tfs2:%" PRIu8 "\tld1:%" PRIu8 "\tlm1:%" PRIu8 "\tls1:%" PRIu8 "\tld2:%" PRIu8 "\tlm2:%"
//            PRIu8 "\tls2:%" PRIu8 "\tbuz:%" PRIu8,
//			tem, hum, adc_val, door1, d_mod, door_state,
//			door2, d_mod_2, door_state_2,
//			fan1, motor_speed, fan2, motor_speed_2,
//			led1, led_mod, led_state,
//			led2, led_mod_2, led_state_2,
//			buz_state);
	Flash_write_info();

	sprintf((char*) tx,
			"tem:%.1f\thum:%.1f\tmq2:%d\tdr1:%d\tdm1:%d\tds1:%d\tdr2:%d\tdm2:%d\tds2:%d\tfn1:%d\tfs1:%d\tfn2:%d\tfs2:%d\tld1:%d\tlm1:%d\tls1:%d\tld2:%d\tlm2:%d\tls2:%d\tbuz:%d",
			tem, hum, adc_val, door1, d_mod, door_state, door2, d_mod_2,
			door_state_2, fan1, motor_speed, fan2, motor_speed_2, led1, led_mod,
			led_state, led2, led_mod_2, led_state_2, buz_state);
	HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
}

void Flash_write_info() {
	sprintf((char*) flash_data, "%d %d %d %d %d %d %d %d %d %d", door1, d_mod,
			door2, d_mod_2, fan1, fan2, led1, led_mod, led2, led_mod_2);
	W25Q32_erase4k(SECTOR_USE * SECTOR_SIZE);
	W25Q32_WriteData(flash_data, SECTOR_USE * SECTOR_SIZE,
			strlen((char*) flash_data));
//	HAL_UART_Transmit(&huart1, flash_data, strlen((char*) flash_data), 200);
}

void Flash_get_info() {
	W25Q32_ReadData(flash_data, SECTOR_USE * SECTOR_SIZE, 19);
//	HAL_UART_Transmit(&huart1, flash_data, strlen((char*) flash_data), 200);
//	sscanf((char*) flash_data, "%d %d %d %d %d %d %d %d %d %d",
//			&door1, &d_mod, &door2, &d_mod_2,
//			&fan1, &fan2, &led1, &led_mod,
//			&led2, &led_mod_2);
	sscanf((char*) flash_data,
			"%hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu", &door1, &d_mod,
			&door2, &d_mod_2, &fan1, &fan2, &led1, &led_mod, &led2, &led_mod_2);
}

void Send_data() {
	if (g_sys_tick - dur_tick >= DURATION) {
		Response();
		dur_tick = g_sys_tick;
	}
}

void Handle_Command() {
	if (g_sys_tick - uart_last_rcv >= 20 && strlen((char*) uart_buf) >= 7) {
		// LIGHT command
		HAL_UART_Transmit(&huart1, uart_buf, strlen((char*) uart_buf), 200);
		if (strlen((char*) uart_buf) == 7
				&& strncmp((char*) uart_buf, "c:l:1:", 6) == 0) {
			led1 = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "LED 1 active: %d\n", led1);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
			Response();
		} else if (strlen((char*) uart_buf) == 7
				&& strncmp((char*) uart_buf, "c:l:2:", 6) == 0) {
			led2 = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "LED 2 active: %d\n", led2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
			Response();
		} else if (strncmp((char*) uart_buf, "c:l:1:m:", 8) == 0
				&& strlen((char*) uart_buf) == 9) {
			l_done = 1;
			led_mod = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "LED 1 mode: %d\n", led_mod);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:l:2:m:", 8) == 0) {
			l_done_2 = 1;
			led_mod_2 = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "LED 2 mode: %d\n", led_mod_2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:l:1:s:", 8) == 0) {
			l_done = 1;
			led_state = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "LED 1 state: %d\n", led_state);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:l:2:s:", 8) == 0) {
			l_done_2 = 1;
			led_state_2 = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "LED 2 state: %d\n", led_state_2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		}

		// FAN command
		else if (strlen((char*) uart_buf) == 7
				&& strncmp((char*) uart_buf, "c:f:1:", 6) == 0) {
			fan1 = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "fan 1 active: %d\n", fan1);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
			Response();
		} else if (strlen((char*) uart_buf) == 7
				&& strncmp((char*) uart_buf, "c:f:2:", 6) == 0) {
			fan2 = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "fan 2 active: %d\n", fan2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
			Response();
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:f:1:s:", 8) == 0) {
			f_done = 1;
			speed = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "fan 1 speed: %d\n", speed);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:f:2:s:", 8) == 0) {
			f_done_2 = 1;
			speed_2 = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "fan 2 speed: %d\n", speed_2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		}

		// DOOR command
		else if (strlen((char*) uart_buf) == 7
				&& strncmp((char*) uart_buf, "c:d:1:", 6) == 0) {
			door1 = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "door 1 active: %d\n", door1);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
			Response();
		} else if (strlen((char*) uart_buf) == 7
				&& strncmp((char*) uart_buf, "c:d:2:", 6) == 0) {
			door2 = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "door 2 active: %d\n", door2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
			Response();
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:d:1:m:", 8) == 0) {
			d_done = 1;
			d_mod = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "door 1 mode: %d\n", d_mod);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:d:2:m:", 8) == 0) {
			d_done_2 = 1;
			d_mod_2 = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "door 2 mode: %d\n", d_mod_2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:d:1:s:", 8) == 0) {
			d_done = 1;
			door_state = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "door 1 state: %d\n", door_state);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strlen((char*) uart_buf) == 9
				&& strncmp((char*) uart_buf, "c:d:2:s:", 8) == 0) {
			d_done_2 = 1;
			door_state_2 = atoi((char*) uart_buf + 8);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "door 2 state: %d\n", door_state_2);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strncmp((char*) uart_buf, "c:b:s:", 6) == 0
				&& strlen((char*) uart_buf) == 7) {
			buz_done = 1;
			buz_state = 0;
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "buz state: %d\n", buz_state);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else if (strncmp((char*) uart_buf, "c:d:s:", 6) == 0
				&& strlen((char*) uart_buf) == 7) {
			isDebug = atoi((char*) uart_buf + 6);
			clear_uart_buf();
			if (isDebug) {
				sprintf((char*) tx, "Debug state: %d\n", isDebug);
				HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
			}
		} else {
			clear_uart_buf();
			sprintf((char*) tx, "Invalid Command!\n");
			HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
		}
	}
}

void control_door() {
	if (d_done) {
		if (d_mod) {
			door_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
			if (door_state)
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 15);
			else
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 25);
		} else {
			if (door_state)
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 15);
			else
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 25);
		}
		d_done = 0;
		Response();
	}
	if (d_done_2) {
		if (d_mod_2) {
			door_state_2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
			if (door_state_2)
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 5);
			else
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 15);
		} else {
			if (door_state_2)
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 5);
			else
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 15);
		}
		d_done_2 = 0;
		Response();
	}
}

void en_motor() {
	if (f_done) {
		if (speed == 0) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
		} else if (speed == 1) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 200);
		} else if (speed == 2) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 400);
		} else if (speed == 3) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 600);
		} else if (speed == 4) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 800);
		} else if (speed == 5) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 999);
		}
		sprintf((char*) tx, "fan 1x speed: %d\n", speed);
		HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
		f_done = 0;
		Response();
	}
	if (f_done_2) {
		if (speed_2 == 0) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
		} else if (speed_2 == 1) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 200);
		} else if (speed_2 == 2) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 400);
		} else if (speed_2 == 3) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 600);
		} else if (speed_2 == 4) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 800);
		} else if (speed_2 == 5) {
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 999);
		}
		f_done_2 = 0;
		Response();
	}
}

void motor_get_speed() {
	HAL_Delay(200);
	en_cnt = __HAL_TIM_GetCounter(&htim4);
	en_cnt_2 = __HAL_TIM_GetCounter(&htim5);
	HAL_Delay(100);
	motor_speed = (__HAL_TIM_GetCounter(&htim4) - en_cnt) * 600 / 384; //encoder cnt increase 384 when complete 1 round
	motor_speed_2 = (__HAL_TIM_GetCounter(&htim5) - en_cnt_2) * 600 / 384;
	sprintf((char*) tx, "\n%d %d\n", motor_speed_2, check_speed(motor_speed_2));
	HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
	if (check_speed(motor_speed) != speed)
		motor_speed = -1;
	else
		motor_speed = speed;
	if (check_speed(motor_speed_2) != speed_2)
		motor_speed_2 = -1;
	else
		motor_speed_2 = speed_2;
}

uint8_t check_speed(int encoder_val) {

	if (encoder_val == 0)
		return 0;
	else if (encoder_val >= 600 && encoder_val < 1150)
		return 1;
	else if (encoder_val >= 1150 && encoder_val < 1950)
		return 2;
	else if (encoder_val >= 1950 && encoder_val < 3000)
		return 3;
	else if (encoder_val >= 3000 && encoder_val < 3900)
		return 4;
	else
		return 5;
}

void control_led() {
	if (l_done) {
		if (led_mod) {
			HAL_Delay(300);
			led_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, led_state);
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, led_state);
		}
		l_done = 0;
		Response();
	}
	if (l_done_2) {
		if (led_mod_2) {
			HAL_Delay(300);
			led_state_2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, led_state_2);
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, led_state_2);
		}
		l_done_2 = 0;
		Response();
	}
}

void Blink_Led() {
	if (g_sys_tick - led_tick >= 1000) {
//		getADC_value();
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		led_tick = g_sys_tick;
	}
}

void warning() {
	if (adc_val < 1000) {
		buz_lvl = 1;
	} else
		buz_lvl = 0;
}

void buzzer() {
	if (buz_done) {
		warning();
		if (buz_state) {
			if (buz_lvl) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
			} else {
				buz_state = 0;
			}
		}
	}
}

void delay_us(uint32_t us) {
	__HAL_TIM_SetCounter(&htim2, 0);
	while (__HAL_TIM_GetCounter(&htim2) < us)
		;
}

void clear_uart_buf() {
	memset(uart_buf, 0, strlen((char*) uart_buf));
	uart_buf_cnt = 0;
}

void init_dht22() {
	gpio_set_mode(OUTPUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
}

void dht22_GetValue(dht22 *dht) {
	uint8_t bytes[5];

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
	delay_us(1000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
	delay_us(20);

	gpio_set_mode(INPUT);

//	delay_us(40);
//	if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
//		sprintf((char*) tx, "Err1");
//		HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
//		tem = -1;
//		hum = -1;
//		return;				// pb9 should be 0
//	}
//
//	delay_us(80);
//
//	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
//		sprintf((char*) tx, "Err2");
//		HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
//		tem = -1;
//		hum = -1;
//		return;				// pb9 should be 1
//	}
//	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));					// wait input become low
	delay_us(120);

	for (int j = 0; j < 5; j++) {
		uint8_t result = 0;
		for (int i = 0; i < 8; i++) { //for each bit in each byte (8 total)
			time_out = g_sys_tick;					// wait input become high
			while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)) {
				if (g_sys_tick - time_out >= DHT_TIMEOUT) {
					if (isDebug) {
						sprintf((char*) tx, "input h");
						HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
					}
					tem = -1;
					hum = -1;
					init_dht22();
					return;
				}
			}
			delay_us(30);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))// if input still high after 30us -> bit 1
				//result |= (1 << (7-i));
				result = (result << 1) | 0x01;
			else
				// else bit 0
				result = result << 1;
			time_out = g_sys_tick;
			while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))	// wait dht22 transmit bit 1 complete
			{
				if (g_sys_tick - time_out >= DHT_TIMEOUT) {
					if (isDebug) {
						sprintf((char*) tx, "tsm cplt");
						HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
					}
					tem = -1;
					hum = -1;
					init_dht22();
					return;
				}
			}
		}
		bytes[j] = result;
	}

	init_dht22();

	dht->hum = bytes[0];
	dht->hum0 = bytes[1];
	dht->tem = bytes[2];
	dht->tem0 = bytes[3];
	dht->check_sum = bytes[4];

	uint16_t check = (uint16_t) bytes[0] + (uint16_t) bytes[1]
			+ (uint16_t) bytes[2] + (uint16_t) bytes[3];
	if ((check % 256) != bytes[4]) {
		if (isDebug) {
			sprintf((char*) tx, "Err cs");
			HAL_UART_Transmit(&huart1, tx, strlen((char*) tx), 200);
		}
		tem = -1;
		hum = -1;
		init_dht22();
		return;							// incorrect checksum
	}

	uint16_t t = ((uint16_t) dht->tem << 8) | ((uint16_t) dht->tem0);
	uint16_t h = ((uint16_t) dht->hum << 8) | ((uint16_t) dht->hum0);

	tem = (float) t / 10;
	hum = (float) h / 10;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	UNUSED(GPIO_Pin);
	if (GPIO_Pin == GPIO_PIN_0) {				// sr501 pin
		if (g_sys_tick - btn_tick >= 500) {
			if (d_mod)
				d_done = 1;
			btn_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_2) {			// sr501 2 pin
		if (g_sys_tick - btn_tick >= 500) {
			if (d_mod_2)
				d_done_2 = 1;
			btn_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_3) {			// light sensor pin
		if (g_sys_tick - lss_tick >= 500) {
			if (led_mod)
				l_done = 1;
			lss_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_1) {			// light sensor 2 pin
		if (g_sys_tick - lss_tick >= 500) {
			if (led_mod_2)
				l_done_2 = 1;
			lss_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_4) {				// door button
		if (g_sys_tick - btn_tick >= 500) {
			d_mod = 0;
			door_state ^= 1;
			d_done = 1;
			btn_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_5) {				// light button
		if (g_sys_tick - btn_tick >= 500) {
			led_mod = 0;
			led_state ^= 1;
			l_done = 1;
			btn_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_15) {				// light button 2
		if (g_sys_tick - btn_tick >= 500) {
			led_mod_2 = 0;
			led_state_2 ^= 1;
			l_done_2 = 1;
			btn_tick = g_sys_tick;
		}
	} else if (GPIO_Pin == GPIO_PIN_8) {				// fan button
		if (g_sys_tick - btn_tick >= 500) {
			f_done = 1;
			speed++;
			if (speed == 6)
				speed = 0;
			btn_tick = g_sys_tick;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
		if (uart_buf_cnt < sizeof(uart_buf)) {
			uart_buf[uart_buf_cnt] = uart_chr;
		}
		uart_last_rcv = g_sys_tick;
		uart_buf_cnt++;
		HAL_UART_Receive_IT(&huart1, &uart_chr, sizeof(uart_chr));
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim3.Instance) {
		g_sys_tick++;
		if (g_sys_tick >= 0xFFFFFFFF)
			g_sys_tick = 0;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == hspi2.Instance) {
		rx_spi_flg = 1;
	}
}

void getADC_value() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	buz_done = 1;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
