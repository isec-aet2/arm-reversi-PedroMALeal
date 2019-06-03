/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {MENU, SINGLE, MULTI, LASTSCORE, SCORE, COLOR, RULES} STATE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_REFRESH_PERIOD   251    /* Internal temperature refresh period */
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define AMBIENT_TEMP            25    /* Ambient Temperature */
#define VSENS_AT_AMBIENT_TEMP  760    /* VSENSE value (mv) at ambient temperature */
#define AVG_SLOPE               25    /* Avg_Solpe multiply by 10 */
#define VREF                  3300

#define DIM   					 8         //dimensao do tabuleiro

#define UPLEFT 					 0        //definiçao de rosa dos ventos para testar as varias direçoes
#define UP 						 1
#define UPRIGHT					 2
#define LEFT					 3
#define RIGHT					 4
#define DOWNLEFT				 5
#define DOWN					 6
#define DOWNRIGHT				 7

#define Opponent				'o'
#define Mine					'm'
#define Empty					'e'
#define Poss					'p'

#define TRUE					 1
#define FALSE					 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
volatile uint8_t updateDisplay=0;
TS_StateTypeDef TS_State;
STATE state;
bool flagTemp=0;
bool flagTS=0;
bool flagMenu=0;
volatile int countTotal=0;
volatile int countPlay=20;
volatile int ScorePlayer1=0;
volatile int ScorePlayer2=0;
volatile int Player = 1;
bool dir[8]={0};
char board[DIM][DIM]={0};
uint32_t color2 = LCD_COLOR_LIGHTMAGENTA;
uint32_t color1 = LCD_COLOR_DARKGREEN;
uint32_t colorboard = LCD_COLOR_LIGHTGRAY;
uint32_t colorposs = LCD_COLOR_DARKGRAY;
unsigned int nBytes;


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7)
	{
		flagTemp=1;
	}

	if(htim->Instance == TIM6)
	{
		updateDisplay=1;
		countTotal++;
		countPlay--;
		if(countPlay==0)
			countPlay=20;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_13){
		BSP_TS_GetState(&TS_State);
		flagTS=1;
		HAL_Delay(50);
	}

	if(GPIO_Pin == GPIO_PIN_0){
		flagMenu=1;
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_SDMMC2_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void LCD_Config1();
static void LCD_Config2();
void displayTemp();
void printMenu();
void touchMenu();
void printBoard();
void InitialPos();
void printTime();
void printScore();
void virtualBoardUpdate();
void clearPrePossMov();
void checkPossMov(int i, int j);
void LCDBoardUpdate();
void readRules();
void ReadFromSD();
void writeGameInfoSD();
void pass3times();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_SDMMC2_SD_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  LCD_Config1();
  HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim7);
  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  BSP_TS_ITConfig();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printMenu();
  state=MENU;

  while (1)
  {

	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  if(flagTemp){
		  displayTemp();
	  }

	  switch (state){

	  case MENU:

		  if(flagTS){
			  LCD_Config2();
			  touchMenu();
			  updateDisplay=1;

			  break;
		  }

	  //case SINGLE:

		  //já lá vamos, semelhante ao multi

	  case MULTI:
		  if(updateDisplay==1)

		  {
			  updateDisplay=0;

			  printBoard();
			  InitialPos();
			  printTime();
			  HAL_TIM_Base_Start_IT(&htim6);
			  printScore();
			  virtualBoardUpdate();
			  clearPrePossMov();
			  LCDBoardUpdate();
			  Player++;
			  break;
		  }

	  case RULES:
		  if(updateDisplay==1)

		  {
			  updateDisplay=0;
			  readRules();
		  }

		  break;
	  }



	  if(flagMenu){
	 		  printMenu();
	 		  state=MENU;
	 		  flagTS=0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SDMMC2
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 20;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_DISABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 640;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 654;
  hltdc.Init.AccumulatedActiveH = 485;
  hltdc.Init.TotalWidth = 660;
  hltdc.Init.TotalHeigh = 487;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC2_Init 0 */

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */

  /* USER CODE END SDMMC2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 19999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin : PI13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PI15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void LCD_Config1(void)
{
	uint32_t  lcd_status = LCD_OK;

	/* Initialize the LCD */
	lcd_status = BSP_LCD_Init();
	while(lcd_status != LCD_OK);

	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);

}
static void LCD_Config2(void)
{

	//BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_BACKGROUND);

	/* Clear the LCD */
	BSP_LCD_Clear(LCD_COLOR_WHITE);

	/* Set LCD Example description */
	HAL_Delay(50);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 40);
	HAL_Delay(50);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"REVERSI by Pedro Leal", CENTER_MODE);
	HAL_Delay(50);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24);
	HAL_Delay(50);
	//BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_FOREGROUND);
}

void displayTemp(){

	uint32_t ConvertedValue;
	long int JTemp;
	char desc[100];

	flagTemp=0;

	HAL_StatusTypeDef status=HAL_ADC_PollForConversion(&hadc1,TEMP_REFRESH_PERIOD); //se o valor aqui obtido estiver ok -> HAL_OK
	if(status==HAL_OK)//so corre quando o valor foi bem recebido e nao por excesso de tempo
	{
		ConvertedValue=HAL_ADC_GetValue(&hadc1); //get value
		JTemp = ((((ConvertedValue * VREF)/MAX_CONVERTED_VALUE) - VSENS_AT_AMBIENT_TEMP) * 10 / AVG_SLOPE) + AMBIENT_TEMP;
		/* Display the Temperature Value on the LCD */
		sprintf(desc, "Temp: %ldC", JTemp);
		//BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_BACKGROUND);
		BSP_LCD_SetFont(&Font20);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)desc, RIGHT_MODE);
		HAL_Delay(50);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	}
}

void printMenu(){

	LCD_Config2();

	if(flagTemp)
		displayTemp();

	flagMenu=0;

	char string[20];

	BSP_LCD_DrawVLine(250, 50, 375);
	BSP_LCD_DrawHLine(250, 50, 300);
	BSP_LCD_DrawHLine(250, 85, 300);
	BSP_LCD_DrawHLine(250, 425, 300);
	BSP_LCD_DrawVLine(550, 50, 375);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_FillRect(250, 50, 300 , 35);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_DARKBLUE);
	sprintf(string, "Menu");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(10,60 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	sprintf(string, "1 vs 1");
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(10,110 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_FillCircle(285, 120, 10);
	sprintf(string, "1 vs PC");
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(10,160 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_FillCircle(285, 170, 10);
	sprintf(string, "Last Score");
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(10,210 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_FillCircle(285, 220, 10);
	sprintf(string, "Hi Score");
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(10,260 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_FillCircle(285, 270, 10);
	sprintf(string, "Color");
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(10,310 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_FillCircle(285, 320, 10);
	sprintf(string, "Rules");
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(10,360 , (uint8_t *)string, CENTER_MODE);
	BSP_LCD_FillCircle(285, 370, 10);
}


void touchMenu(){

	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=110 && TS_State.touchY[0]<=150)
	{
		state = SINGLE;
		HAL_Delay(50);
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=160 && TS_State.touchY[0]<=200)
	{
		state = MULTI;
		HAL_Delay(50);
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=210 && TS_State.touchY[0]<=250)
	{
		state = LASTSCORE;
		HAL_Delay(50);
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=260 && TS_State.touchY[0]<=300)
	{
		state = SCORE;
		HAL_Delay(50);
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=310 && TS_State.touchY[0]<=350)
	{
		state = COLOR;
		HAL_Delay(50);
	}
	if(TS_State.touchX[0]>=250 && TS_State.touchX[0]<=550 && TS_State.touchY[0]>=360 && TS_State.touchY[0]<=400)
	{
		state = RULES;
		HAL_Delay(50);
	}

	flagTS=0;
}

void printBoard(){

	//LCD_Config2();

	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
	BSP_LCD_FillRect(50, 50, 50*8, 50*8);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	for(int i = 0; i<=9; i++){
		BSP_LCD_DrawVLine((BSP_LCD_GetXSize()/16)*i, BSP_LCD_GetYSize()/10, 400);
	}
	for(int j = 0; j<=9; j++){
		BSP_LCD_DrawHLine(BSP_LCD_GetXSize()/16, (BSP_LCD_GetYSize()/9.6)*j, 400);
	}
}

void InitialPos(){
	//Posições iniciais
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
	BSP_LCD_FillCircle(225,225, 20);
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);
	BSP_LCD_FillCircle(275, 275, 20);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
	BSP_LCD_FillCircle(275, 225, 20);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
	BSP_LCD_FillCircle(225, 275, 20);

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
}

void printTime(){

	char string[50];

	int h=countTotal/3600, m=(countTotal-3600*h)/60, s=(countTotal-3600*h-m*60);
	sprintf(string, "Time Total: %2d:%2d:%2d", h,m,s);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize()/2 + 120, (uint8_t *)string, RIGHT_MODE);

	sprintf(string, "Time of Play: %d s", countPlay);
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_DisplayStringAt(20, BSP_LCD_GetYSize()/2 + 145, (uint8_t *)string, RIGHT_MODE);
}

void printScore(){

	char string[35];
	//BSP_LCD_Clear(LCD_COLOR_WHITE);

	BSP_LCD_DrawVLine(460, 50, 400);
	BSP_LCD_DrawHLine(460, 50, 330);
	BSP_LCD_DrawHLine(460, 85, 330);
	BSP_LCD_DrawHLine(460, 450, 330);
	BSP_LCD_DrawVLine(790, 50, 400);
	BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

	sprintf(string, "Game Info");
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(90,55 , (uint8_t *)string, RIGHT_MODE);

	sprintf(string, "Player 1 Score: %d", ScorePlayer1);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(30,200 , (uint8_t *)string, RIGHT_MODE);

	sprintf(string, "Player 2 Score: %d", ScorePlayer2);
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayStringAt(30,255 , (uint8_t *)string, RIGHT_MODE);
}

void virtualBoardUpdate(){

	uint32_t col1, col2;

	if (Player%2 == 1){ // se for player 1
			col1 = color1;
			col2 = color2;
		}
		else{
			col1 = color2;
			col2 = color1;
		}

	for (int i = 0; i < DIM; i++){
		for (int j = 0; j < DIM; j++){
			if(BSP_LCD_ReadPixel(75+50*i,75+50*j)==color1)
				board[i][j]=Mine;
			if(BSP_LCD_ReadPixel(75+50*i,75+50*j)==color2)
				board[i][j]=Opponent;
			if(BSP_LCD_ReadPixel(75+50*i,75+50*j)==colorboard || BSP_LCD_ReadPixel(75+50*i,75+50*j)==colorposs)
				board[i][j]=Empty;
		}
	}
}


void clearPrePossMov(){

	for(int i = 0; i < DIM; i++){
		for(int j = 0; j < DIM; j++){
			if(board[i][j]=Poss)
			{
				board[i][j]=Empty;
			}
			checkPossMov(i,j);
		}
	}
}

void checkPossMov(int i, int j){


	int iaux, jaux;

	iaux = i - 1;       //Testing UP+LEFT
	jaux = j - 1;
	while (board[iaux][jaux] == Opponent) { 				//enquanto a posiçao que estamos a verificar estiver dentro dos
		iaux--;                                                         //limites do tabuleiro e for igual ao simbolo do adversario
		jaux--;                                                         //continuamos a movimentar o nosso estudo na direçao a testar

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i - 1;       //Testing UP
	jaux = j;
	while (board[iaux][jaux] == Opponent) {
		iaux--;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i - 1;       //Testing UP+RIGHT
	jaux = j + 1;
	while (board[iaux][jaux] == Opponent) {
		iaux--;
		jaux++;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i;       //Testing LEFT
	jaux = j - 1;
	while (board[iaux][jaux] == Opponent) {
		jaux--;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i;       //Testing RIGHT
	jaux = j + 1;
	while (board[iaux][jaux] == Opponent) {
		jaux++;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i + 1;       //Testing DOWN+LEFT
	jaux = j - 1;
	while (board[iaux][jaux] == Opponent) {
		iaux++;
		jaux--;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i + 1;       //Testing DOWN
	jaux = j;
	while (board[iaux][jaux] == Opponent) {
		iaux++;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}


	iaux = i + 1;       //Testing DOWN+RIGHT
	jaux = j + 1;
	while (board[iaux][jaux] == Opponent) {
		iaux++;
		jaux++;

		if (board[iaux][jaux] == Mine) {
			if (board[i][j] == Empty) {                             //substitui espaço em branco por jogada possivel
				board[i][j] = Poss;                                //assinalada por * (estrelinha)
				dir[UPLEFT] = TRUE;                                 //True quando o teste de direçao funcionou, sera usado na funçao reverse
			}
		}
	}
}

void LCDBoardUpdate(){

	for (int i = 0; i < DIM; i++){
		for (int j = 0; j < DIM; j++){
			if(board[i][j]==Mine){
				BSP_LCD_SetTextColor(color1);
				BSP_LCD_FillCircle(75+50*i,75+50*j, 20);
			}
			else if(board[i][j]==Opponent){
				BSP_LCD_SetTextColor(color2);
				BSP_LCD_FillCircle(75+50*i,75+50*j, 20);
			}
			else if(board[i][j]==Empty){
				BSP_LCD_SetTextColor(colorboard);
				BSP_LCD_FillCircle(75+50*i,75+50*j, 20);
			}
			else if(board[i][j]==Poss){
				BSP_LCD_SetTextColor(colorposs);
				BSP_LCD_FillCircle(75+50*i,75+50*j, 5);
			}
		}
	}
}

/*void blockMove (uint16_t *cX, uint16_t *cY)
{
	if(BSP_LCD_ReadPixel(cX,cY)==PossMov){
		putCircle(cX, cY);
	}
}
*/

void readRules(){

    LCD_Config2();

    char stringRules[200];
    sprintf(stringRules, "RULES");
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(15,100 , (uint8_t *)stringRules, CENTER_MODE);
    sprintf(stringRules, "Reversi is a strategy board game for two players,");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,180 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "played on an 8 x 8 uncheckered board.");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,200 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "There are sixty-four identical game pieces called disks (often spelled discs),");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,220 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "which are light on one side and dark on the other.");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,240 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "Players take turns placing disks on the board with their assigned color facing up.");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,260 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "During a play, any disks of the opponent's color that are in a straight line and bounded by the disk");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,280 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "just placed and another disk of the current player's color are turned over to the current player's color.");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,300 , (uint8_t *)stringRules, LEFT_MODE);
    sprintf(stringRules, "The object of the game is to have the majority of disks turned to display your color when the last playable empty square is filled.");
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_DisplayStringAt(8,320 , (uint8_t *)stringRules, LEFT_MODE);
}

void ReadFromSD(){

	char string[100];

    if(f_mount (&SDFatFS, SDPath, 0)!=FR_OK) //activa o sistema
    {
        Error_Handler();
    }
    HAL_Delay(100);
    if(f_open (&SDFile, "Score.txt", FA_READ)!=FR_OK){ //tenta criar ficheiro em modo escrita e testa se detectou SD
        Error_Handler();
    }
    HAL_Delay(100);

    if(f_read (&SDFile, string, sizeof(string), &nBytes)!=FR_OK){ // strlen(string) ->numero de caracteres sizeof->n de bytes
        Error_Handler();
    }

    BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)string, CENTER_MODE);
    HAL_Delay(100);
    f_close (&SDFile);
}

void writeGameInfoSD(){

	char string[100];

    if(f_mount (&SDFatFS, SDPath, 0)!=FR_OK){ //activa o sistema
        Error_Handler();
    }
    HAL_Delay(100);
    if(f_open (&SDFile, "Score.txt", FA_OPEN_APPEND | FA_WRITE)!=FR_OK){ //tenta criar ficheiro em modo escrita e testa se detectou SD
        Error_Handler();
    }
    HAL_Delay(100);

  //  sprintf(string,"Vencedor: %s, Jogador 1 Pontuação: %d, Jogador 1 Pontuação: %d, Tempo de jogo: %d, Temp média: &%ld\n", stringPlayer, ScorePlayer1, ScorePlayer2, counter, Tempave);

    int x=strlen(string)*sizeof(char);

    if(f_write (&SDFile, string, strlen(x), &nBytes)!=FR_OK) // strlen(string) ->numero de caracteres sizeof->n de bytes
        Error_Handler();
    HAL_Delay(100);
    f_close (&SDFile);
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
	BSP_LED_Toggle(LED_RED);
	HAL_Delay(500);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
