/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "icm20948.h"
#include <stdlib.h>
#include <stdio.h>
#include "const.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for showTask */
osThreadId_t showTaskHandle;
const osThreadAttr_t showTask_attributes = {
  .name = "showTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
  .name = "encoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ultraTask */
osThreadId_t ultraTaskHandle;
const osThreadAttr_t ultraTask_attributes = {
  .name = "ultraTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for icm20948_task */
osThreadId_t icm20948_taskHandle;
const osThreadAttr_t icm20948_task_attributes = {
  .name = "icm20948_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void motor(void *argument);
void encoder(void *argument);
void ultra(void *argument);
void uart(void *argument);
void icm20948(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//initializing global variables
uint8_t aRxBuffer[50]; //for taking in stuff from rpi thru UART
uint8_t ch[10] = "Complete\n";
double pwmVal_L = 2000; // pwm values
double pwmVal_R = 2000;
#define DEFAULTPWM 5000
uint8_t dir = 1; // controls dir of wheels, 1 for front, 0 for back
uint8_t servoVal = 150; //set servo dir
double duration=0; //duration in ms
double reqduration=0, curduration=0;
uint8_t distprof = 0;
uint8_t servodefault = 150;
uint8_t correctionleft = 147, correctionright = 152;

//Ultrasonic
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

//For Profile Switching
uint8_t userBtnCount = 0;
uint8_t enableSw = 0;

//For Gyro
/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948            0xD0
#define I2C_ADD_ICM20948_AK09916    0x0C
#define I2C_ADD_ICM20948_AK09916_READ  0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00
/* define ICM-20948 Register */
/* user bank 0 register */
#define REG_ADD_WIA             0x00
#define REG_VAL_WIA             0xEA
#define REG_ADD_USER_CTRL       0x03
#define REG_VAL_BIT_DMP_EN          0x80
#define REG_VAL_BIT_FIFO_EN         0x40
#define REG_VAL_BIT_I2C_MST_EN      0x20
#define REG_VAL_BIT_I2C_IF_DIS      0x10
#define REG_VAL_BIT_DMP_RST         0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1     0x06
#define REG_VAL_ALL_RGE_RESET   0x01
#define REG_VAL_DEVICE_RESET    0x41	//self added
#define REG_VAL_RUN_MODE        0x01    //Non low-power mode
#define REG_ADD_LP_CONFIG       0x05
#define REG_ADD_PWR_MGMT_1      0x06
#define REG_ADD_PWR_MGMT_2      0x07
#define REG_VAL_DISABLE_GYRO    0x07
#define REG_VAL_ENABLE_ALL      0x00
#define REG_ADD_ACCEL_XOUT_H    0x2D
#define REG_ADD_ACCEL_XOUT_L    0x2E
#define REG_ADD_ACCEL_YOUT_H    0x2F
#define REG_ADD_ACCEL_YOUT_L    0x30
#define REG_ADD_ACCEL_ZOUT_H    0x31
#define REG_ADD_ACCEL_ZOUT_L    0x32
#define REG_ADD_GYRO_XOUT_H     0x33
#define REG_ADD_GYRO_XOUT_L     0x34
#define REG_ADD_GYRO_YOUT_H     0x35
#define REG_ADD_GYRO_YOUT_L     0x36
#define REG_ADD_GYRO_ZOUT_H     0x37
#define REG_ADD_GYRO_ZOUT_L     0x38
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_REG_BANK_SEL    0x7F
#define REG_VAL_REG_BANK_0  0x00
#define REG_VAL_REG_BANK_1  0x10
#define REG_VAL_REG_BANK_2  0x20
#define REG_VAL_REG_BANK_3  0x30

/* user bank 1 register */
/* user bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1   0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]   */
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */

/* user bank 3 register */
#define REG_ADD_I2C_SLV0_ADDR   0x03
#define REG_ADD_I2C_SLV0_REG    0x04
#define REG_ADD_I2C_SLV0_CTRL   0x05
#define REG_VAL_BIT_SLV0_EN     0x80
#define REG_VAL_BIT_MASK_LEN    0x07
#define REG_ADD_I2C_SLV0_DO     0x06
#define REG_ADD_I2C_SLV1_ADDR   0x07
#define REG_ADD_I2C_SLV1_REG    0x08
#define REG_ADD_I2C_SLV1_CTRL   0x09
#define REG_ADD_I2C_SLV1_DO     0x0A

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */
#define REG_ADD_MAG_WIA1    0x00
#define REG_VAL_MAG_WIA1    0x48
#define REG_ADD_MAG_WIA2    0x01
#define REG_VAL_MAG_WIA2    0x09
#define REG_ADD_MAG_ST2     0x10
#define REG_ADD_MAG_DATA    0x11
#define REG_ADD_MAG_CNTL2   0x31
#define REG_VAL_MAG_MODE_PD     0x00
#define REG_VAL_MAG_MODE_SM     0x01
#define REG_VAL_MAG_MODE_10HZ   0x02
#define REG_VAL_MAG_MODE_20HZ   0x04
#define REG_VAL_MAG_MODE_50HZ   0x05
#define REG_VAL_MAG_MODE_100HZ  0x08
#define REG_VAL_MAG_MODE_ST     0x10
/* define ICM-20948 MAG Register  end */


/* ICM20948 Variables --------------------------------------------------------*/
uint8_t icmData = 0x1;
uint8_t icmTempMsg[16] = {};
ICM20948_ST_SENSOR_DATA gstGyroOffset ={0,0,0};
IMU_ST_SENSOR_DATA gstMagOffset = {0,0,0};
int16_t TestmagnBuff[9]={0};
int16_t magn[3];
short Deviation_gyro[3],Original_gyro[3];
short s16Gyro[3], s16Accel[3], s16Magn[3];
int16_t gyro[3], accel[3],magnet[3];
//uint16_t Deviation_Count = 0;
uint64_t gyrosum=0, gyrozero=0;
int64_t gyrosumsigned=0;

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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  HAL_UART_Transmit_IT(&huart3, (uint8_t *) aRxBuffer, 50);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of showTask */
  showTaskHandle = osThreadNew(show, NULL, &showTask_attributes);

  /* creation of motorTask */
  motorTaskHandle = osThreadNew(motor, NULL, &motorTask_attributes);

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(encoder, NULL, &encoderTask_attributes);

  /* creation of ultraTask */
  ultraTaskHandle = osThreadNew(ultra, NULL, &ultraTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(uart, NULL, &uartTask_attributes);

  /* creation of icm20948_task */
  icm20948_taskHandle = osThreadNew(icm20948, NULL, &icm20948_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_SW_Pin */
  GPIO_InitStruct.Pin = ENABLE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENABLE_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//User functions
void UpdateDeviation()
{
	memcpy(Deviation_gyro, gyro, sizeof(gyro));
}

void ProcessGyro()
{
	if(abs(gyro[2]-gyrozero)>10){
			  gyrosum+=abs(gyro[2]-gyrozero);
			  gyrosumsigned+=gyro[2]-gyrozero;
		  }
}

int16_t gyro_z()
{
	return gyro[2];
}

int64_t Gyrosumsigned()
{
	return gyrosumsigned;
}

void resetgyrosumsigned()
{
	gyrosumsigned = 0;
}
void InitMotor(uint8_t dir)
{
	// set motor directions
	if (dir == FORWARD_DIR)  // forward
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}
	else if (dir == BACKWARD_DIR)
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	}
}

uint32_t AdjustPWM(uint32_t pwmPrev, double ppmsActual, double ppmsTarget)
{
	uint32_t pwm = pwmPrev;
	if (ppmsTarget - ppmsActual > PPMS_THRESHOLD)
	{
		pwm += (ppmsTarget - ppmsActual) * PPMS_TO_PWM_RATIO;
	}
	else if (ppmsActual - ppmsTarget > PPMS_THRESHOLD)
	{
		pwm -= (ppmsActual - ppmsTarget) * PPMS_TO_PWM_RATIO;
	}
	pwm = pwm < PWM_MIN ? PWM_MIN : pwm;
	pwm = pwm > PWM_MAX ? PWM_MAX : pwm;
	return pwm;
}

void StraightMovement(
		uint8_t dir,
		uint32_t servo,
		uint32_t pwmL,
		uint32_t pwmR,
		uint32_t pulseTarget,
		double ppmsTarget,  // target pulse per millisecond
		uint8_t willTransmit
		)
{
	char oledBuffer1[20], oledBuffer2[20], oledBuffer3[20], oledBuffer4[20];
	uint32_t pulseTotal = 0, pulseTotalL = 0, pulseTotalR = 0;
	uint32_t timeStampPrev, timeStampCurr, timeStampDiff;
	uint32_t pulsePrevL, pulseCurrL, pulseDiffL;
	uint32_t pulsePrevR, pulseCurrR, pulseDiffR;
	double ppmsL, ppmsR;

	InitMotor(dir);  // initialize motors
	htim1.Instance->CCR4 = servo;  // set default servo value
	osDelay(200);

	pulsePrevL = __HAL_TIM_GET_COUNTER(&htim2);  // get initial pulse
	pulsePrevR = __HAL_TIM_GET_COUNTER(&htim3); // get initial pulse

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);  // start motor L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmL);  // start motor R
	timeStampPrev = HAL_GetTick();  // get initial time stamp

//	StartSum();
	gyrosumsigned = 0;

	// Movement loop
	while (pulseTotal < pulseTarget)
	{
		osDelay(FORWARD_DELAY);  // delta time

		timeStampCurr = HAL_GetTick();
		pulseCurrL = __HAL_TIM_GET_COUNTER(&htim2);
		pulseCurrR = __HAL_TIM_GET_COUNTER(&htim3);

		timeStampDiff = timeStampCurr - timeStampPrev;
		timeStampPrev = timeStampCurr;

		// Update delta pulse
		// FORWARD: left pulse --, right pulse ++
		// BACKWARD: left pulse ++, right pulse --
		if (dir == FORWARD_DIR)
		{
			pulseDiffL = (pulseCurrL <= pulsePrevL) ? pulsePrevL - pulseCurrL : (65535 - pulseCurrL) + pulsePrevL + 1;
			pulseDiffR = (pulseCurrR >= pulsePrevR) ? pulseCurrR - pulsePrevR : (65535 - pulsePrevR) + pulseCurrR + 1;
		}
		else  // backward
		{
			pulseDiffL = (pulseCurrL >= pulsePrevL) ? pulseCurrL - pulsePrevL : (65535 - pulsePrevL) + pulseCurrL + 1;
			pulseDiffR = (pulseCurrR <= pulsePrevR) ? pulsePrevR - pulseCurrR : (65535 - pulseCurrR) + pulsePrevR + 1;
		}
		pulsePrevL = pulseCurrL;
		pulsePrevR = pulseCurrR;

		ppmsL = pulseDiffL * 1.0 / timeStampDiff;
		ppmsR = pulseDiffR * 1.0 / timeStampDiff;

		// Adjust PWM
		if ((ppmsL < ppmsTarget && ppmsL < ppmsR) || (ppmsL > ppmsTarget && ppmsL > ppmsR))
		{
			pwmL = AdjustPWM(pwmL, ppmsL, ppmsTarget);
		}
		if ((ppmsR < ppmsTarget && ppmsR < ppmsL) || (ppmsR > ppmsTarget && ppmsR > ppmsL))
		{
			pwmR = AdjustPWM(pwmR, ppmsR, ppmsTarget);
		}

		pulseTotalL += pulseDiffL;
		pulseTotalR += pulseDiffR;
		pulseTotal = (pulseTotalL + pulseTotalR) / 2;

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmL);  // update
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmR);  // update

		int8_t sign = dir == FORWARD_DIR ? 1 : -1;

		if (gyrosumsigned * sign > 180)
		{
			gyrosumsigned = 0;
			servo = servo + 1 <= SERVO_MAX ? servo + 1 : servo;
			servo++;
			htim1.Instance->CCR4 = servo;
//			StartSum();
		}
		else if (gyrosumsigned * sign < -180)
		{
			gyrosumsigned = 0;
			servo = servo - 1 > SERVO_MIN ? servo - 1 : servo;
			servo--;
			htim1.Instance->CCR4 = servo;
//			StartSum();
		}

		// Display
//		sprintf(oledBuffer1, "PWM%5lu/%5lu", pwmL, pwmR);
//		sprintf(oledBuffer2, "PPM%5.2f/%5.2f", ppmsL, ppmsR);
//		sprintf(oledBuffer3, "PUL%5ld/%5ld", pulseTotalL, pulseTotalR);
//		sprintf(oledBuffer4, "SER%5ld/%5ld", servo, pulseTarget);
//		OLED_ShowString(0,  0, (uint8_t*) oledBuffer1);
//		OLED_ShowString(0, 12, (uint8_t*) oledBuffer2);
//		OLED_ShowString(0, 24, (uint8_t*) oledBuffer3);
//		OLED_ShowString(0, 36, (uint8_t*) oledBuffer4);
	}
	// Stop motors
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	realignWheels();

	if(willTransmit == 1){
		HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
	}



//	ResetSum();
}

uint32_t DistanceToPulse(int distance)
{
	uint32_t pulse = ((PULSE_PER_REVOLUTION * distance))/ WHEEL_C - FORWARD_SLIDE_PULSE;
	return pulse;
}

uint32_t SmallDistanceToPulse(int distance)
{
	uint32_t pulse = ((PULSE_PER_REVOLUTION * distance) *0.95)/ SMALL_WHEEL_C - SMALL_FORWARD_SLIDE_PULSE;
	return pulse;
}

void Straight(uint8_t dir, int distance, uint8_t willTransmit, uint32_t servo)
{
	uint32_t pulseTarget = DistanceToPulse(distance);
	StraightMovement(dir, servo, DEFAULT_FORWARD_PWM_L, DEFAULT_FORWARD_PWM_R, pulseTarget, FORWARD_PULSE_PER_MS, willTransmit);
}

void SmallStraight(uint8_t dir, int distance, uint8_t willTransmit, uint32_t servo)
{
	uint32_t pulseTarget = SmallDistanceToPulse(distance);
	StraightMovement(dir, servo, DEFAULT_FORWARD_PWM_L, DEFAULT_FORWARD_PWM_R, pulseTarget, FORWARD_PULSE_PER_MS , willTransmit);
}

void userFunctions(){}
void resetSTM(){
	HAL_NVIC_SystemReset(); //try this
	wdg_activate(10);
	wdg_reactivate();
}
void realignWheels(){
//	htim1.Instance->CCR4 = 220; //right
//	osDelay(400);
	htim1.Instance->CCR4 = DEFAULT_FORWARD_SERVO; //left
	osDelay(600);
//	htim1.Instance->CCR4 = ; //center
}

//void motorActivate(double pwmVal_L, double pwmVal_R, uint8_t dir, float duration, uint8_t servoVal){
void motorActivate(){
//	duration = 2363; //1m
	if(dir == 1){
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
	}
	htim1.Instance->CCR4 = servoVal;
//	osDelay(400);

	uint32_t tick = HAL_GetTick();

	double pwml = pwmVal_L;
	double pwmr = pwmVal_R;

	while((HAL_GetTick()-tick)<=(duration)){
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwml);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmr);
	}

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,0);
}

void straight(double local_pwmVal_L, double local_pwmVal_R, uint8_t local_dir, double distance){
	pwmVal_L = local_pwmVal_L/2; // pwm values
	pwmVal_R = local_pwmVal_R/2;
	dir = local_dir;
	servoVal = 150; //set servo dir
	duration = distance * 70 *0.415;
	motorActivate();
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
}

void straight2(double local_pwmVal_L, double local_pwmVal_R, uint8_t local_dir, double distance){
	pwmVal_L = local_pwmVal_L/2; // pwm values
	pwmVal_R = local_pwmVal_R/2;
	dir = local_dir;
	servoVal = 150; //set servo dir
	duration = distance * 70 *0.415;
	motorActivate();
//	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
}

void gyrostraight(double local_pwmVal_L, double local_pwmVal_R, uint8_t local_dir, double distance){
	pwmVal_L = local_pwmVal_L; // pwm values
	pwmVal_R = local_pwmVal_R;
	dir = local_dir;
	reqduration = distance * 33 *0.415;
	curduration = 0;
	gyrosumsigned = 0;
	servodefault = 150;
	while(curduration<=(reqduration)*0.8){
		servoVal = servodefault;
		if(dir==1){
			if(gyrosumsigned>250){ //going left
				//servoVal = correctionright;
				servodefault+=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else if(gyrosumsigned<-250){ //going right
				//servoVal = correctionright;
				servodefault-=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else{
				duration = 30;
				motorActivate();
			}
		}else{
			if(gyrosumsigned>500){ //going left
				//servoVal = correctionright;
				servodefault-=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else if(gyrosumsigned<-500){ //going right
				//servoVal = correctionright;
				servodefault+=1;
				duration = 30;
				motorActivate();
				gyrosumsigned=0;
			}else{
				duration = 30;
				motorActivate();
			}
		}

		sprintf(icmTempMsg,"G:%+07d,",gyro[2]);
		HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);

		curduration+=30;
	}
	while(curduration<=reqduration){
		servoVal = 150;
		pwmVal_L = 1800;
				pwmVal_R = 2000;
				duration = 30;
				motorActivate();


		sprintf(icmTempMsg,"G:%+07d,",gyro[2]);
		HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);

		curduration+=30;
	}
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
}

void turn(uint8_t local_dir, int leftright, int angle){
	dir = local_dir;
	distprof=2;
	if(distprof==1){ //red tiles/////////////////////////////////////////////////////////////
		if(dir==1){ //forward
			if(leftright == 0){//left
				if(angle==15) duration = 127;
				if(angle==45) duration = 399;
				//        if(angle==15) duration = 105;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,0.7); //move forward by 0.7cm
					//duration = 815; //for actual red tiles
//					duration = 805;
					duration = 790;

				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 112; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,0.6); //move forward by 0.6cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,2.1); //move forward by 2.1cm
				}
			}else{//right
				//duration = 7.604762*angle + 40;
				if(angle==15) duration = 110;
				if(angle==45) duration = 355;
				//        if(angle==15) duration = 101;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,3.1);//move forward by 3.1cm
					//duration =720;//lab
//					duration =715;//red tiles
					duration = 700;
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,4.5); //move forward by 4.5cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,2); //move forward by 2cm
				}
			}
		} else {//backward
			if(leftright == 0){//left
				if(angle==15) duration = 127;
				if(angle==45) duration = 378.5;
				if(angle == 90)  {//making the forward turning cover 25cm by 40cm
					straight(DEFAULTPWM,DEFAULTPWM,0,1.5);//move back by 1.5 cm
					duration =757; //duration =750;
				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 112; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,0,4.2); //move backwards by 4.2cm
					straight(DEFAULTPWM,DEFAULTPWM,0,4.7); //move backwards by 4.2cm
				}
			}else{//right
				if(angle==15) duration = 127;
				if(angle==45) duration = 377.5;
				if(angle == 90)  {
					//straight(DEFAULTPWM,DEFAULTPWM,0,0);  //move back by 0.8cm
					duration =755;
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
					straight(DEFAULTPWM,DEFAULTPWM,0,3.1); //move back by 4.6cm
				}
			}
		}
	}//end red tiles
	else if(distprof==2){ //lab floor/////////////////////////////////////////////////////////////
		if(dir==1){ //forward
			if(leftright == 0){//left
				if(angle==15) duration = 140;
				if(angle==45) duration = 420;
				//        if(angle==15) duration = 105;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,0.7); //move forward by 0.7cm
					duration = 850;
				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 103; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,0.6); //move forward by 0.6cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,2.1); //move forward by 2.1cm
				}
			}else{//right
				//duration = 7.604762*angle + 40;
				if(angle==15) duration = 129;
				if(angle==45) duration = 385;
				//        if(angle==15) duration = 101;
				if(angle == 90)  {//making the forward turning cover 40cm by 25cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,3.1);//move forward by 3.1cm
					duration =770;//red tiles
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,1,4.5); //move forward by 4.5cm
					//straight(DEFAULTPWM,DEFAULTPWM,1,2); //move forward by 2cm
				}
			}
		} else {//backwards
			if(leftright == 0){//left
				if(angle==15) duration = 137;
				if(angle==45) duration = 410;
				if(angle == 90)  {//making the forward turning cover 25cm by 40cm
					//straight(DEFAULTPWM,DEFAULTPWM,0,1.5);//move back by 1.5 cm
					duration =820; //duration =750;
				}
					pwmVal_L = 2400; // pwm values
					pwmVal_R = 4800;
					servoVal = 100; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
//					straight(DEFAULTPWM,DEFAULTPWM,0,4.2); //move backwards by 4.2cm
					//straight(DEFAULTPWM,DEFAULTPWM,0,4.7); //move backwards by 4.2cm
				}
			}else{//right
				if(angle==15) duration = 139;
				if(angle==45) duration = 420;
				if(angle == 90)  {
					//straight(DEFAULTPWM,DEFAULTPWM,0,0);  //move back by 0.8cm
					duration =840;
				}
					pwmVal_L = 4800; // pwm values
					pwmVal_R = 2400;
					servoVal = 230; //set servo dir
//					osDelay(1000);
					motorActivate();
					realignWheels();
				if(angle == 90)  {
					//straight(DEFAULTPWM,DEFAULTPWM,0,3.1); //move back by 4.6cm
				}
			}
		}
	}//end lab
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);


}

void gyroturn(uint8_t local_dir, int leftright, int angle){ //23,580 for 90 degrees
	gyrosum = 0;
	dir = local_dir;
	if(leftright==0) htim1.Instance->CCR4 = 105;
	else htim1.Instance->CCR4 = 210;
	osDelay(300);
	while (gyrosum<=(angle*245*4*0.7)){
			if(leftright==0){//left
				duration = 10;
				pwmVal_L = 1200; // pwm values 1200
				pwmVal_R = 2400; // 2400
				servoVal = 100; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}else{//right
				duration = 10;
				pwmVal_L = 2400; // pwm values
				pwmVal_R = 0;
				servoVal = 290; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}
	}
	while (gyrosum<=(angle*245*4)){ //last 30%
			if(leftright==0){//left
				duration = 10;
				pwmVal_L = 600; // pwm values
				pwmVal_R = 1200;
				servoVal = 100; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}else{//right
				duration = 10;
				pwmVal_L = 1200; // pwm values
				pwmVal_R = 0;
				servoVal = 290; //set servo dir
				motorActivate();
//				sprintf(icmTempMsg,"%+09d,",gyrosum);
//				HAL_UART_Transmit(&huart3, (uint8_t *)&icmTempMsg, 10, 0xFFFF);
			}
	}
}

void tpturn(int leftright){
	//three 30 turns
	int dur;
	//switch(angle){

	if(leftright==1){
		turn(1,1,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 16.12);

		turn(1,1,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 23.442);

		turn(1,1,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 9);
	}else{
		turn(1,0,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 16.12);

		turn(1,0,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 23.442);

		turn(1,0,30);
		straight(DEFAULTPWM, DEFAULTPWM, 0, 9);
	}

}

void ipt90(int leftright){
	if (leftright ==0){//left
//		turn(1,0,15);
//		turn(0,1,15);
//		turn(1,0,15);
//		turn(0,1,15);
//		turn(1,0,15);
//		turn(0,1,15);

		//Straight(1,3.3);
		//straight(DEFAULTPWM, DEFAULTPWM, 0, 2.3+1);
		SmallStraight(1, 8, 0, 150);
		osDelay(100);
//		straight2(4500,4500,0,9);
		gyroturn(1,0,42);
		osDelay(100);
		gyroturn(0,1,44);
		realignWheels();
		osDelay(100);
		SmallStraight(0, 4, 0, 140);
		osDelay(100);
		//Straight(0,3);
		//straight2(4500,4500,1,3); //forward 3
		//straight(DEFAULTPWM, DEFAULTPWM, 1, 4.7+1);
		//Straight(0,5.7);
	}else{
//		turn(1,1,15);
//		turn(0,0,15);
//		turn(1,1,15);
//		turn(0,0,15);
//		turn(1,1,15);
//		turn(0,0,15);
		//straight(DEFAULTPWM, DEFAULTPWM, 0, 4.6+1);

		//Straight(1,8);
//		straight2(4500,4500,0,2);
		SmallStraight(1, 4, 0, 150);
		osDelay(100);
		gyroturn(1,1,45);
		osDelay(100);
		gyroturn(0,0,41); //OR 41
		osDelay(100);
		SmallStraight(0, 6, 0, 160);
		osDelay(100);
//		straight2(4500,4500,1,7.5);
		//Straight(0,7,5);
		//straight(DEFAULTPWM, DEFAULTPWM, 1, 0.7+1);
	}
	realignWheels();
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
}

void ICMWriteOneByte(uint8_t RegAddr, uint8_t Data)
{
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADD_ICM20948, RegAddr, I2C_MEMADD_SIZE_8BIT, &Data, 1, 0xffff);
}

uint8_t ICMReadOneByte(uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	HAL_I2C_Mem_Read(&hi2c1, I2C_ADD_ICM20948, RegAddr, I2C_MEMADD_SIZE_8BIT, &TempVal, 1, 0xffff);
	return TempVal;
}

void ICMReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    ICMWriteOneByte(REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV0_REG,  u8RegAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = ICMReadOneByte(REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);
    osDelay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);

    for(i=0; i<u8Len; i++)
    {
        *(pu8data+i) = ICMReadOneByte(REG_ADD_EXT_SENS_DATA_00+i);

    }
    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = ICMReadOneByte(REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL,  u8Temp);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

}

void ICMWriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    ICMWriteOneByte(REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3); //swtich bank3
    ICMWriteOneByte(REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_REG,  u8RegAddr);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_DO,   u8data);
    ICMWriteOneByte(REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    u8Temp = ICMReadOneByte(REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);
    osDelay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    ICMWriteOneByte(REG_ADD_USER_CTRL, u8Temp);

    ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3); //swtich bank3

    u8Temp = ICMReadOneByte(REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    ICMWriteOneByte(REG_ADD_I2C_SLV0_CTRL,  u8Temp);

   ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //swtich bank0

    return;
}

void ICMWhoIAm()
{
	uint8_t ICM_OK_Msg[8] = "ICM OK";
	if (REG_VAL_WIA == ICMReadOneByte(REG_ADD_WIA))
	{
		OLED_ShowString(0,20,ICM_OK_Msg);
	}
}

void ICMCalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
	uint8_t i;

	*(pAvgBuffer + ((*pIndex) ++)) = InVal;
  	*pIndex &= 0x07;

  	*pOutVal = 0;
	for(i = 0; i < 8; i ++)
  	{
    	*pOutVal += *(pAvgBuffer + i);
  	}
  	*pOutVal >>= 3;
}

void ICMGyroRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_XOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_XOUT_H);
    s16Buf[0] =	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_YOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_YOUT_H);
    s16Buf[1] =	(u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0] = ICMReadOneByte(REG_ADD_GYRO_ZOUT_L);
    u8Buf[1] = ICMReadOneByte(REG_ADD_GYRO_ZOUT_H);
    s16Buf[2] =	(u8Buf[1]<<8)|u8Buf[0];

#if 1
    for(i = 0; i < 3; i ++)
    {
        ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
#else
    *ps16X = s16Buf[0];
    *ps16Y = s16Buf[1];
    *ps16Z = s16Buf[2];
#endif
    return;
}

void ICMAccelRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t u8Buf[2];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_XOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_XOUT_H);
   s16Buf[0] = (u8Buf[1]<<8)|u8Buf[0];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_YOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_YOUT_H);
   s16Buf[1] = (u8Buf[1]<<8)|u8Buf[0];

   u8Buf[0] = ICMReadOneByte(REG_ADD_ACCEL_ZOUT_L);
   u8Buf[1] = ICMReadOneByte(REG_ADD_ACCEL_ZOUT_H);
   s16Buf[2] = (u8Buf[1]<<8)|u8Buf[0];

#if 1
   for(i = 0; i < 3; i ++)
   {
       ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }
   *ps16X = s32OutBuf[0];
   *ps16Y = s32OutBuf[1];
   *ps16Z = s32OutBuf[2];

#else
   *ps16X = s16Buf[0];
   *ps16Y = s16Buf[1];
   *ps16Z = s16Buf[2];
#endif
   return;

}

void ICMMagRead(int16_t* ps16X, int16_t* ps16Y, int16_t* ps16Z)
{
   uint8_t counter = 20;
   uint8_t u8Data[MAG_DATA_LEN];
   int16_t s16Buf[3] = {0};
   uint8_t i;
   int32_t s32OutBuf[3] = {0};
   static ICM20948_ST_AVG_DATA sstAvgBuf[3];
   while( counter>0 )
   {
       osDelay(10);
       ICMReadSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_ST2, 1, u8Data);

       if ((u8Data[0] & 0x01) != 0)
           break;

       counter--;
   }

   if(counter != 0)
   {
       ICMReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                   REG_ADD_MAG_DATA,
                                   MAG_DATA_LEN,
                                   u8Data);
       s16Buf[0] = ((int16_t)u8Data[1]<<8) | u8Data[0];
       s16Buf[1] = ((int16_t)u8Data[3]<<8) | u8Data[2];
       s16Buf[2] = ((int16_t)u8Data[5]<<8) | u8Data[4];
   }
   else
   {
       printf("\r\n Mag is busy \r\n");
   }
#if 1
   for(i = 0; i < 3; i ++)
   {
       ICMCalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
   }

   *ps16X =  s32OutBuf[0];
   *ps16Y = -s32OutBuf[1];
   *ps16Z = -s32OutBuf[2];
#else
   *ps16X = s16Buf[0];
   *ps16Y = -s16Buf[1];
   *ps16Z = -s16Buf[2];
#endif
   return;
}

void ICMGyroOffset()
{
	uint8_t i;
    int16_t	s16Gx = 0, s16Gy = 0, s16Gz = 0;
	int32_t	s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    for(i = 0; i < 32; i ++)
 	{
        ICMGyroRead(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
		s32TempGy += s16Gy;
		s32TempGz += s16Gz;
        osDelay(10);
    }
    gstGyroOffset.s16X = s32TempGx >> 5;
	gstGyroOffset.s16Y = s32TempGy >> 5;
	gstGyroOffset.s16Z = s32TempGz >> 5;
    return;
}

void MPU_Get_Gyroscope(void)
{
	ICMGyroRead(&gyro[0], &gyro[1], &gyro[2]);
	//if(Deviation_Count==CONTROL_DELAY)
	//{
		//Save the raw data to update zero by clicking the user button
		Original_gyro[0] = gyro[0];
		Original_gyro[1] = gyro[1];
		Original_gyro[2] = gyro[2];

		//Removes zero drift data
		gyro[0] = Original_gyro[0]-Deviation_gyro[0];
		gyro[1] = Original_gyro[1]-Deviation_gyro[1];
		gyro[2] = Original_gyro[2]-Deviation_gyro[2];
	//}
	return;
}

void ICMInit()
{
	/* user bank 0 register */
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
	//ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
	ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_DEVICE_RESET);
	osDelay(10);
	ICMWriteOneByte(REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);


	/* user bank 2 register */

	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);

	ICMWriteOneByte(REG_ADD_GYRO_SMPLRT_DIV, 0x07);
	ICMWriteOneByte(REG_ADD_GYRO_CONFIG_1,REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);

	ICMWriteOneByte(REG_ADD_ACCEL_SMPLRT_DIV_2,  0x07);
	ICMWriteOneByte(REG_ADD_ACCEL_CONFIG,REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);

	/* user bank 0 register */
	ICMWriteOneByte(REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

	osDelay(100);
	/* offset */
	ICMGyroOffset();

	ICMWriteSecondary(I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==htim4.Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(&htim4, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//prevent unused argument(s) compilation warning
	UNUSED(huart);
	HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 20);
	HAL_UART_Transmit(&huart3,(uint8_t *)aRxBuffer, 20, 0xFFFF);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  for(;;)
  {
	  if(HAL_GPIO_ReadPin(GPIOD, USER_BTN_Pin)==0){
		  if(userBtnCount<13){
			  userBtnCount++;
			  osDelay(100);
		  }else{
			  userBtnCount=0;
			  osDelay(100);
		  }
	  }


	  osDelay(100);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the showTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  /* Infinite loop */
//	uint8_t hello[20] = "";
	uint8_t disp[20] = "";
	uint8_t disp2[20] = "";

	for(;;)
	{




	sprintf(disp2,"%s",aRxBuffer);
	OLED_ShowString(0,40,disp2);

//Ultrasonic Distance reading
	sprintf(disp,"Dist:%5d",Distance);
	OLED_ShowString(0,30,disp);
	OLED_Refresh_Gram();

//print current floor type

  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
	//initialize timers
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	realignWheels();

	/* Infinite loop */
	int count=0;
	int reset=1;
	for(;;)
	{
//		if(reset==1){
//			realignWheels();
//			reset=0;
//		}
//
//
		osDelay(100);

	}
	osDelay(200);
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  /* Infinite loop */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //MOTORA Encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //MOTORB Encoder

  int tim2Cnt1, tim2Cnt2, tim2Diff; //initiate some counters and a diff variable
  int tim3Cnt1, tim3Cnt2, tim3Diff;
  int expected;
  uint32_t tick;

  tim2Cnt1 = __HAL_TIM_GET_COUNTER(&htim2); //Get count at rising edge
  tim3Cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
  tick = HAL_GetTick(); //Grab current tick and slap it into tick

  uint8_t disp[20];
  uint16_t dir2, dir3;


  for(;;)
  {
	if(HAL_GetTick()-tick > 1000L){
		tim2Cnt2 = __HAL_TIM_GET_COUNTER(&htim2); //get updated no of counts
		tim3Cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
		//
		//tim2
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
			if(tim2Cnt2<tim2Cnt1) //underflow
				tim2Diff = tim2Cnt1 - tim2Cnt2;
			else
				tim2Diff = (65535 - tim2Cnt2)+tim2Cnt1;
		}
		else{
			if(tim2Cnt2 > tim2Cnt1)
				tim2Diff = tim2Cnt2 - tim2Cnt1;
			else
				tim2Diff = (65536 - tim2Cnt1)+tim2Cnt2;
		}
		//
		//tim3 doesnt work
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
			if(tim3Cnt2<tim3Cnt1) //underflow
				tim3Diff = tim3Cnt1 - tim3Cnt2;
			else
				tim3Diff = (65535 - tim3Cnt2)+tim3Cnt1;
		}
		else{
			if(tim3Cnt2 > tim3Cnt1)
				tim3Diff = tim3Cnt2 - tim3Cnt1;
			else
				tim3Diff = (65536 - tim3Cnt1)+tim3Cnt2;
		}

		sprintf(disp,"LSpeed:%5d",tim2Diff);
		OLED_ShowString(0,0,disp);
		sprintf(disp,"RSpeed:%5d",tim3Diff);
		OLED_ShowString(0,10,disp);

		expected = (tim2Diff+tim3Diff)/2;
		if(tim2Diff>expected){
			pwmVal_L - 15;
		}else if(tim2Diff<expected){
			pwmVal_L + 15;
		}
		if(tim3Diff>expected){
			pwmVal_R - 15;
		}else if(tim3Diff<expected){
			pwmVal_R + 15;
		}

//		dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
//		dir3 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
//		sprintf(disp,"Direct:%5d",dir2);
//		OLED_ShowString(0,20,disp);
//		sprintf(disp,"RDir:%5d",dir3);
//		OLED_ShowString(0,30,disp);
		//repeat
		tim2Cnt1 = __HAL_TIM_GET_COUNTER(&htim2); //Get count at rising edge
	    tim3Cnt1 = __HAL_TIM_GET_COUNTER(&htim3);
	    tick = HAL_GetTick(); //Grab current time and slap it into tick

	}
    //osDelay(1);
  }
  osDelay(100);
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_ultra */
/**
* @brief Function implementing the ultraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultra */
void ultra(void *argument)
{
  /* USER CODE BEGIN ultra */
  /* Infinite loop */

	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
  for(;;)
  {
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	osDelay(1);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

    osDelay(10);
  }
  /* USER CODE END ultra */
}

/* USER CODE BEGIN Header_uart */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uart */
void uart(void *argument)
{
  /* USER CODE BEGIN uart */
  /* Infinite loop */

  for(;;)
  {
	  //if (userBtnCount == 8){
	  	  HAL_UART_Receive_IT(&huart3, (uint8_t *) aRxBuffer, 20);


		  char * pch = malloc(20);
		  pch = strtok (aRxBuffer," ");

		  if(strcmp(pch, "RESET")==0){
			  HAL_NVIC_SystemReset();
			  osDelay(100); //adjust this maybe
			  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);

		  }
		  if(strcmp(pch, "FORWARD")==0){
			  dir=1;
		  }else{
			  dir=0;
		  }

		  pch = strtok (NULL, " ");

		  if(strcmp(pch,"TURN")==0){ //IF TURN
			  pch = strtok (NULL, " "); //Next word first
			  if(strcmp(pch,"LEFT")==0){//IF TURN LEFT
				  //if (dir=1) //straight(DEFAULTPWM,DEFAULTPWM,1,50);
				  ipt90(0); //shud be close to 10cm turn radius
				  //gyroturn(dir,0,90);
				  //gyroturn(dir, 0, 90);
				  //straight(DEFAULTPWM, DEFAULTPWM, 1, 10);
				  //HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
			  }
			  if(strcmp(pch,"LEFFT")==0){//IF TURN LEFT 180
				  gyroturn(dir,0,180);
			  }
			  if(strcmp(pch,"LEFFFT")==0){//IF TURN LEFT 270
				  gyroturn(dir,0,270);
			  }
			  if(strcmp(pch,"RIGHT")==0){//IF TURN RIGHT
				  //turn2(dir);

				  ipt90(1); // 10cm turn radius
				  //gyroturn(dir,1,90);
				  //gyroturn(dir,1,90);
				  //HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 10, 0xFFFF);
			  }
			  if(strcmp(pch,"RIGHHT")==0){//IF TURN RIGHT 180
			  				  gyroturn(dir,1,180);
			  			  }
			  if(strcmp(pch,"RIGHHHT")==0){//IF TURN RIGHT 270
			  				  gyroturn(dir,1,270);
			  			  }
		  }
		  else if(strcmp(pch,"MOVE")==0){
			  pch = strtok (NULL, " "); //Next word first
			  if (dir==0){
//				  straight(4500,4500,dir,atoi(pch));
				  Straight(BACKWARD_DIR, atoi(pch), 1, 150);
			  }else{
				  Straight(FORWARD_DIR, atoi(pch), 1, 150);
				  //gyrostraight(4500,DEFAULTPWM,dir,atoi(pch));
				  //gyrostraight(2250,2500,dir,atoi(pch));
				  //gyrostraight(5000,DEFAULTPWM,dir,atoi(pch));

			  }

		  }

		  for(int i =0; i<20;i++){
			  sprintf(aRxBuffer[i], ' ');
		  } //????? DOES THIS EVEN WORK

    osDelay(100);
  }
  /* USER CODE END uart */
}

/* USER CODE BEGIN Header_icm20948 */
/**
* @brief Function implementing the icm20948_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_icm20948 */
void icm20948(void *argument)
{
  /* USER CODE BEGIN icm20948 */
	uint8_t mag_count = 0;
	uint32_t lastWakeTime = HAL_GetTick();
	ICMInit(); //initialize ICM
  /* Infinite loop */
  for(;;)
  {
	  //vTaskDelayUntil(&lastWakeTime, 0.01);
//	  if(Deviation_Count<CONTROL_DELAY)
	  {
//		  Deviation_Count++;
		  memcpy(Deviation_gyro,gyro,sizeof(gyro));
	  }
	  //Get acceleration sensor data
//	  ICMAccelRead(&accel[0], &accel[1], &accel[2]);
	  //Get gyroscope data
	  MPU_Get_Gyroscope();
	  //Get magnetometer data
//	  ICMMagRead(&magnet[0], &magnet[1], &magnet[2]);
//	  sprintf(icmTempMsg,"%d %d %d      ",magnet[0],magnet[1],magnet[2]);
	  sprintf(icmTempMsg,"Gyr%+07d",gyro[2]);
	  OLED_ShowString(0, 20, icmTempMsg);
	  if(abs(gyro[2]-gyrozero)>50){
		  gyrosum+=abs(gyro[2]-gyrozero);
		  gyrosumsigned+=gyro[2]-gyrozero;
	  }



	  osDelay(25);
  }
  /* USER CODE END icm20948 */
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
