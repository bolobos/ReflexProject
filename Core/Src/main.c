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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rgb_lcd.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <time.h>
 //#include "cmsis_os2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NBR_COL 18

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GereLeds */
osThreadId_t GereLedsHandle;
const osThreadAttr_t GereLeds_attributes = {
  .name = "GereLeds",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GereLcdScreen */
osThreadId_t GereLcdScreenHandle;
const osThreadAttr_t GereLcdScreen_attributes = {
  .name = "GereLcdScreen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gereRGB_LCD */
osThreadId_t gereRGB_LCDHandle;
const osThreadAttr_t gereRGB_LCD_attributes = {
  .name = "gereRGB_LCD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gerePoten_AD0 */
osThreadId_t gerePoten_AD0Handle;
const osThreadAttr_t gerePoten_AD0_attributes = {
  .name = "gerePoten_AD0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for personnalDelays */
osThreadId_t personnalDelaysHandle;
const osThreadAttr_t personnalDelays_attributes = {
  .name = "personnalDelays",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for stateDiagram */
osThreadId_t stateDiagramHandle;
const osThreadAttr_t stateDiagram_attributes = {
  .name = "stateDiagram",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for potenValue */
osMessageQueueId_t potenValueHandle;
const osMessageQueueAttr_t potenValue_attributes = {
  .name = "potenValue"
};
/* Definitions for semaphoreTest */
osSemaphoreId_t semaphoreTestHandle;
const osSemaphoreAttr_t semaphoreTest_attributes = {
  .name = "semaphoreTest"
};
/* Definitions for semaphoreMenuTimerTemp */
osSemaphoreId_t semaphoreMenuTimerTempHandle;
const osSemaphoreAttr_t semaphoreMenuTimerTemp_attributes = {
  .name = "semaphoreMenuTimerTemp"
};
/* USER CODE BEGIN PV */

enum Etats
{
    STARTING,
    MENU,
    TEST_1,
    TEST_2,
    TEST_3,
    SCORES
};

enum Etats etatCourant = STARTING;

int ignoreSemaphore = 1;
uint16_t potenValue = 0;
char potenValue_string[20];

uint16_t soundSensorValue = 0;
char soundSensorValue_string[20];

int potenValueTemp = 0;

int idGame = 0;

uint8_t fullcharacter[] = {
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };

uint8_t emptycharacter[] = {
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000
    };

// var for the firsts passages
int tempFirst = 0;

int bestRecord = 0;
char bestRecord_string[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void GereLeds_1(void *argument);
void GereLcdScreen_1(void *argument);
void changeRGBs(void *argument);
void getValueAdc(void *argument);
void gereDelays(void *argument);
void gereStateDiagram(void *argument);

/* USER CODE BEGIN PFP */

void startingLCD(void);
void menuLCD(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void displayTest1(void){
    ignoreSemaphore = 1;
    LCD_clear();

    LCD_home();
    LCD_setCursor(0, 0);
    LCD_print("      JEU 1     ", sizeof("HUMAN SPEED TEST")-1);
    LCD_setCursor(0, 1);
    LCD_print("                ", sizeof("HUMAN SPEED TEST")-1);
    osDelay(1000);
    LCD_setCursor(0, 0);
    LCD_print("        3       ", sizeof("HUMAN SPEED TEST")-1);
    osDelay(1000);
    LCD_setCursor(0, 0);
    LCD_print("        2       ", sizeof("HUMAN SPEED TEST")-1);
    osDelay(1000);
    LCD_setCursor(0, 0);
    LCD_print("        1       ", sizeof("HUMAN SPEED TEST")-1);
    osDelay(1000);
    LCD_setCursor(0, 0);
    LCD_print("      WAIT      ", sizeof("HUMAN SPEED TEST")-1);
    osDelay(1000);


    int cpt;
    srand(time(NULL));   // Initialization, should only be called once.
    int r = rand();      // Returns a pseudo-random integer between 0 and RAND_MAX.
    r = r % 5000000;

    char aye_string[16];
    int aye_res;
    bestRecord = 0;
    TIM2->CNT = 0;
    while(r >= cpt){
        cpt = (bestRecord*1000000)+((TIM2->CNT)/4);
    }
    LCD_setCursor(0, 0);
    LCD_print("     PUSH       ", sizeof("HUMAN SPEED TEST")-1);
    bestRecord = 0;
    TIM2->CNT = 0;
    cpt = 0;
    ignoreSemaphore = 0;
    int iVerif = osSemaphoreAcquire(semaphoreTestHandle, 5000);
    if(iVerif == osOK){

        cpt = (bestRecord*1000000)+((TIM2->CNT)/4);
        aye_res = cpt;

        sprintf(aye_string, "%d", aye_res);
        LCD_setCursor(0,1);
        LCD_print(aye_string, sizeof("     Press      ")-1);
        osDelay(5000);

    }
    else if(iVerif == osErrorTimeout){
        LCD_setCursor(0,1);
        LCD_print("TROP LONG FRERO", sizeof("TROP LONG FRERO"));
        osDelay(5000);
    }
}

void displayScores(void){
    while(1){
        LCD_setCursor(0, 0);
        LCD_print(" MEILLEUR SCORE ", sizeof("HUMAN SPEED TEST")-1);
        osDelay(500);
        LCD_setCursor(0, 1);
        sprintf(bestRecord_string, "%d", bestRecord);
        LCD_print(bestRecord_string, sizeof(bestRecord_string)-1);
        osDelay(500);
    }
}

void startingLCD(void){

    LCD_clear();
    LCD_home();
    LCD_setCursor(0, 0);
    LCD_print("HUMAN SPEED TEST", sizeof("HUMAN SPEED TEST")-1);
    //LCD_setCursor(0, 1);
    osDelay(1000);

    LCD_createChar(0,fullcharacter);
    LCD_createChar(1,emptycharacter);

    for (int iBcl = 0; iBcl <= 19; ++iBcl) {
        LCD_setCursor(iBcl,1);
        LCD_write(255);
        if(iBcl>=3){
            LCD_setCursor(iBcl-3,1);
            LCD_write(1);
        }
        osDelay(30);
    }

    LCD_setCursor(0,1);
    LCD_print("    SAE 2023    ", sizeof("    SAE 2023    ")-1);
    osDelay(1000);

    for (int iBcl = 0; iBcl <= 19; ++iBcl) {
        LCD_setCursor(iBcl,1);
        LCD_write(255);
        if(iBcl>=3){
            LCD_setCursor(iBcl-3,1);
            LCD_write(1);
        }
        osDelay(30);
    }

    LCD_setCursor(0,1);
    LCD_print("-- JOS MAR DAV--", sizeof("-- JOS MAR DAV--")-1);
    osDelay(1000);

    for (int iBcl = 0; iBcl <= 24; ++iBcl) {


        if (iBcl >=8){
            LCD_setCursor(iBcl-8,0);
            LCD_write(1);
            LCD_setCursor(iBcl-8,1);
            LCD_write(1);

        }

        if (iBcl <=16){
            LCD_setCursor(iBcl,0);
            LCD_write(255);
            LCD_setCursor(iBcl,1);
            LCD_write(255);
        }


        osDelay(30);
    }
    LCD_setCursor(0,0);
    LCD_print("     Press      ", sizeof("     Press      ")-1);
    LCD_setCursor(0,1);
    LCD_print("  to continue   ", sizeof("  to continue   ")-1);
}

void menuLCD(void){

    /*if (tempFirst == 0){
        LCD_createChar(0,fullcharacter);
        LCD_setCursor(1,0);
        LCD_print("     MENU     ", 15);

        LCD_setCursor(0,0);
        LCD_write(0);
        LCD_setCursor(1,0);
        LCD_write(0);
        LCD_setCursor(0,16);
        LCD_write(0);
        LCD_setCursor(1,16);
        LCD_write(0);
        tempFirst++;
    }*/



    osMessageQueueGet(potenValueHandle, &potenValue, 0, osWaitForever);

    if (abs((potenValue - potenValueTemp)) > 10){
        potenValueTemp = potenValue;
        if (potenValue < 1024){
            LCD_setCursor(0,0);
            LCD_print("     JEU 1      ", sizeof("   X  0  0  0   "));
            LCD_setCursor(0,1);
            LCD_print("   X  0  0  0   ", sizeof("   X  0  0  0   "));
            idGame = 1;
        }
        else if (potenValue < 2048){
            LCD_setCursor(0,1);
            LCD_print("   0  X  0  0   ", sizeof("   0  0  0  X   "));
            LCD_setCursor(0,0);
            LCD_print("     JEU 2      ", sizeof("   0  0  0  X   "));
            idGame = 2;
        }
        else if (potenValue < 3072){
            LCD_setCursor(0,1);
            LCD_print("   0  0  X  0   ", sizeof("   0  0  0  X   "));
            LCD_setCursor(0,0);
            LCD_print("     JEU 3      ", sizeof("   0  0  0  X   "));
            idGame = 3;
        }
        else {
            LCD_setCursor(0,1);
            LCD_print("   0  0  0  X   ", sizeof("   0  0  0  X   "));
            LCD_setCursor(0,0);
            LCD_print("     SCORES     ", sizeof("   0  0  0  X   "));
            idGame = 4;
        }
        osDelay(1);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    //LCD_begin(16, 2, 1);
    //LCD_print("AB", 2);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semaphoreTest */
  semaphoreTestHandle = osSemaphoreNew(1, 0, &semaphoreTest_attributes);

  /* creation of semaphoreMenuTimerTemp */
  semaphoreMenuTimerTempHandle = osSemaphoreNew(1, 0, &semaphoreMenuTimerTemp_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of potenValue */
  potenValueHandle = osMessageQueueNew (16, sizeof(uint16_t), &potenValue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GereLeds */
  GereLedsHandle = osThreadNew(GereLeds_1, NULL, &GereLeds_attributes);

  /* creation of GereLcdScreen */
  GereLcdScreenHandle = osThreadNew(GereLcdScreen_1, NULL, &GereLcdScreen_attributes);

  /* creation of gereRGB_LCD */
  gereRGB_LCDHandle = osThreadNew(changeRGBs, NULL, &gereRGB_LCD_attributes);

  /* creation of gerePoten_AD0 */
  gerePoten_AD0Handle = osThreadNew(getValueAdc, NULL, &gerePoten_AD0_attributes);

  /* creation of personnalDelays */
  personnalDelaysHandle = osThreadNew(gereDelays, NULL, &personnalDelays_attributes);

  /* creation of stateDiagram */
  stateDiagramHandle = osThreadNew(gereStateDiagram, NULL, &stateDiagram_attributes);

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


    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // USELESS TEMPS REEL

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GereLeds_1 */
/**
 * @brief Function implementing the GereLeds thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GereLeds_1 */
void GereLeds_1(void *argument)
{
  /* USER CODE BEGIN GereLeds_1 */
    int button = 1;
    int buttonRelease = 1;
    /* Infinite loop */
    for (;;) {

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1) {
            buttonRelease = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
        }
        else {
            if (buttonRelease == 1) {
                buttonRelease = 0;
                if(ignoreSemaphore != 1)
                osSemaphoreRelease(semaphoreTestHandle);
            }
            button = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
        }



        osDelay(1);

    }
  /* USER CODE END GereLeds_1 */
}

/* USER CODE BEGIN Header_GereLcdScreen_1 */
/**
 * @brief Function implementing the GereLcdScreen thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GereLcdScreen_1 */
void GereLcdScreen_1(void *argument)
{
  /* USER CODE BEGIN GereLcdScreen_1 */


    //Etats = STARTING;


    /* Infinite loop */
    for (;;) {
        osDelay(1);
        /*switch (etatCourant) {
            case STARTING:
                for (int var = 0; var <= 16; ++var) {
                    LCD_clear();
                    osDelay(200);
                }
                break;
            case MENU:
                LCD_clear();
                /*if(){

                }
                else if (){

                }
                else {

                }
                break;
            case TEST_1:
                LCD_clear();

                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                LCD_print("Marius chibre", sizeof("Marius chibre"));

                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                LCD_clear();
                break;
            case TEST_2:
                LCD_clear();

                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                LCD_print("Marius chibre", sizeof("Marius chibre"));

                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                LCD_clear();
                break;
            case TEST_3:
                LCD_clear();

                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                LCD_print("Marius chibre", sizeof("Marius chibre"));

                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                LCD_clear();
                break;
            default:
                break;
        }*/

        /*for (int var = 0; var <= 16; ++var) {
            for (int var = 0; var <= 16; ++var) {
                LCD_clear();
                if(){

                }
                osDelay(200);
            }
        }*/





            /*if(){

            }
            ignoreSemaphore = 0;
            osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);

            LCD_clear();*/



            //LCD_print(0, 4);

            /*HAL_ADC_Start(&hadc2);
            HAL_ADC_PollForConversion(&hadc2,20);
            soundSensorValue = HAL_ADC_GetValue(&hadc2);
            sprintf(soundSensorValue_string, "%d", soundSensorValue_string);
            LCD_setCursor(0,0);
            LCD_print(soundSensorValue_string);
            osDelay(100);
            LCD_createChar(0,emptyCharacter);*/

        /*LCD_setCursor(0,0);
        int g = sizeof("      MENU       ");
        char yo[20];
        sprintf(yo, "%d", g);

        LCD_print(yo, sizeof(yo)-1);*/

        ///////////////////////////////////////////////////////////////////////////s


        ///////////////////////////////////////////////////////////////////////////

        /*LCD_setCursor(0,0);
        LCD_print(potenValue_string, 4);
        osDelay(100);
        LCD_createChar(0,emptyCharacter);*/

        /*void LCD_print(char* data, uint8_t size){
            while(size>0){
                LCD_write(data[0]);
                data++;
                size--;
            }
        }*/

        //LCD_write(255);
        //LCD_write(255);
        //LCD_leftToRight();
        //LCD_print("Marius chibre", sizeof("Marius chibre")-1);
    }
  /* USER CODE END GereLcdScreen_1 */
}

/* USER CODE BEGIN Header_changeRGBs */
/**
* @brief Function implementing the gereRGB_LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_changeRGBs */
void changeRGBs(void *argument)
{
  /* USER CODE BEGIN changeRGBs */
      int R = 0;
      int G = 100;
      int B = 200;
      int temp1 = 0;
      int temp2 = 0;
      int temp3 = 0;
  /* Infinite loop */
  for(;;)
  {

      void defilColors(){
          if (temp1 == 0){
                if(R == 254){
                    temp1 = 1;
                }
                R++;
              }
              else{
                if(R == 1){
                    temp1 = 0;
                }
                R--;
              }

              if (temp2 == 0){
                if(G == 254){
                    temp2 = 1;
                }
                G++;
              }
              else{
                if(G == 1){
                    temp2 = 0;
                }
                G--;
              }

              if (temp3 == 0){
                if(B == 254){
                    temp3 = 1;
                }
                B++;
              }
              else{
                if(B == 1){
                    temp3 = 0;
                }
                B--;
              }

              LCD_setRGB(R,G,B);
              osDelay(10);
      }
      defilColors();


  }
  /* USER CODE END changeRGBs */
}

/* USER CODE BEGIN Header_getValueAdc */
/**
* @brief Function implementing the gerePoten_AD0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getValueAdc */
void getValueAdc(void *argument)
{
  /* USER CODE BEGIN getValueAdc */
  /* Infinite loop */
  for(;;)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 20);
    potenValue = HAL_ADC_GetValue(&hadc1);
    sprintf(potenValue_string, "%d", potenValue);
    osMessageQueuePut(potenValueHandle,&potenValue,0,0);
    osDelay(1);
    //HAL_ADC_stop

  }
  /* USER CODE END getValueAdc */
}

/* USER CODE BEGIN Header_gereDelays */
/**
* @brief Function implementing the personnalDelays thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gereDelays */
void gereDelays(void *argument)
{
  /* USER CODE BEGIN gereDelays */
  /* Infinite loop */
  for(;;)
  {
      /*if(abs((potenValue - potenValueTemp)) > 20){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
          osSemaphoreRelease(semaphoreMenuTimerTempHandle);
      }*/
    osDelay(500);
  }
  /* USER CODE END gereDelays */
}

/* USER CODE BEGIN Header_gereStateDiagram */
/**
* @brief Function implementing the stateDiagram thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gereStateDiagram */
/* USER CODE END Header_gereStateDiagram */
void gereStateDiagram(void *argument)
{
  /* USER CODE BEGIN gereStateDiagram */
  /* Infinite loop */
    LCD_begin(16, 2, 0);
    HAL_TIM_Base_Start_IT(&htim2);
  for(;;)
  {
      switch (etatCourant) {
            case STARTING:

                /*menuLCD();
                osDelay(1);*/
                ignoreSemaphore = 1;
                startingLCD();
                ignoreSemaphore = 0;
                osSemaphoreAcquire(semaphoreTestHandle, osWaitForever);
                etatCourant = MENU;

                break;
            case MENU:
                //LCD_clear();
                menuLCD();

                if(osSemaphoreAcquire(semaphoreTestHandle, 1) == osOK){
                    switch(idGame){
                        case 1:
                            etatCourant = TEST_1;
                            break;
                        case 2 :
                            etatCourant = TEST_2;
                            break;
                        case 3 :
                            etatCourant = TEST_3;
                            break;
                        case 4 :
                            etatCourant = SCORES;
                            break;
                        default :
                            break;
                        }
                }


                break;
            case TEST_1:
                displayTest1();

                break;
            case TEST_2:

                break;
            case TEST_3:

                /*osDelay(1);
                sprintf(bestRecord_string, "%d", bestRecord);
                LCD_setCursor(0,1);
                LCD_print(bestRecord_string, sizeof(bestRecord_string)-1);
                osDelay(500);*/
                break;
            case SCORES:
                displayScores();
                break;
            default:
                break;
      }
      osDelay(1);
  }
  /* USER CODE END gereStateDiagram */
}

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
  if (htim->Instance == htim2.Instance)
  {
      bestRecord++;
      osDelay(1);
  }

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
    while (1) {}
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
