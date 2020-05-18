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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADNS3080.h"

uint8_t txdata[1]={0x02};
uint8_t rxdatab[7];
uint8_t rxdataj[7];
uint8_t txbuff[1]={0x50};
uint8_t buff[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t motion_burst[7];
volatile HAL_StatusTypeDef state;
int szamlalo=0;
int i=0;
float dXb=0;
float dYb=0;
float dXj=0;
float dYj=0;
float uart_txdatab[2];
float uart_txdataj[2];
float datab[7];
float dataj[7];
signed char xb=0;
signed char yb=0;
signed char xj=0;
signed char yj=0;
/*
uint8_t Rxdata[8];
uint8_t incr_X;
uint8_t incr_Y;

uint16_t CPI;
uint8_t uartTxBuffer[7];
*/
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
struct {
	float dx;
	float dy;

}uart_txdata;
*/
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
  SROM_download();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	  	  HAL_Delay(250);

	  		uartTxBuffer[0]=255;
	  		uartTxBuffer[1]=255;
	  		uartTxBuffer[2]=255;
*/

	  	 //SROM uploaded
	  		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1); //LD6
	 /* //Motion burst
	  		HAL_GPIO_WritePin(ADNS3080_CS_PORT, ADNS3080_CS_PIN,0);
	  		my_delay();
	  		HAL_SPI_Transmit(&hspi2, txdata, 1, 10);
	  		my_delay();
	  		HAL_SPI_Receive(&hspi2, rxdata, 7, 10);
	  		my_delay();
	  		HAL_GPIO_WritePin(ADNS3080_CS_PORT, ADNS3080_CS_PIN,1);
	  		my_delay();
	          */
	  	     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
	  	 	 HAL_Delay(5);
	  	 	 HAL_SPI_Transmit(&hspi2, txbuff, 1, 10);
	  	 	 HAL_Delay(75);
	  	 	 HAL_SPI_Receive(&hspi2, rxdatab, 7, 10);
	  	 	 //state=HAL_SPI_TransmitReceive(&hspi2, txbuff, rxdatab, 7, 10);
	  	 	 HAL_Delay(5);
	  	 	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	  	 	 HAL_Delay(5);

	  	 	ADNS3080MotionDatab.Motion=rxdatab[0];
	  	 	ADNS3080MotionDatab.DeltaX=rxdatab[1];
	  	 	ADNS3080MotionDatab.DeltaY=rxdatab[2];
	  	 	ADNS3080MotionDatab.SQUAL=rxdatab[3];
	  	 	ADNS3080MotionDatab.ShutterUpper=rxdatab[4];
	  	 	ADNS3080MotionDatab.ShutterLower=rxdatab[5];
	  	 	ADNS3080MotionDatab.MaximumPixel=rxdatab[6];
           		 xb=rxdatab[1];
          		 yb=rxdatab[2];

	  	 				/*if(xb&0x80)
	  	 		  	 	  {

	  	 		  	 	  //Az x kétkomplementális átalakítása
	  	 		  	 	  xb -= 1;
	  	 		  	 	  xb = ~xb;
	  	 		  	 	  xb=(-1)*xb;
	  	 		  	 	  xb-=256;

	  	 		  	 	}
	  	 				SumX=SumX+xb;*/
	  	 	dXb+=(((xb*0.00635)*5)/0.1); // az dXb értéket kell számoltatni addig amíg müködik a progi
	  	 	dYb+=(((yb*0.00635)*5)/0.1);
	  	 	szamlalo+=dXb sqrt
	 		
			int seged = dXb;
			dxb = dxb * dxb;
			dxb = dxb / seged;
	  		
	  		uart_txdatab[0]=dXb;
	  	 	uart_txdatab[1]=dYb;

	  	 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1); //LD5
	  	 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	  	 	HAL_Delay(5);
	  	 	HAL_SPI_Transmit(&hspi2, txbuff, 1, 10);
	  	 	HAL_Delay(150);
	  	 	HAL_SPI_Receive(&hspi2, rxdataj, 7, 10);
	  	 	 //state=HAL_SPI_TransmitReceive(&hspi2, txbuff, rxdataj, 7, 10);
	  	 	HAL_Delay(5);
	  	 	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	  	 	HAL_Delay(5);
	  	 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,1); //LD4



	  	 	ADNS3080MotionDataj.Motion=rxdataj[0];
	  	 	ADNS3080MotionDataj.DeltaX=rxdataj[1];
	  	 	ADNS3080MotionDataj.DeltaY=rxdataj[2];
	  	 	ADNS3080MotionDataj.SQUAL=rxdataj[3];
	  	 	ADNS3080MotionDataj.ShutterUpper=rxdataj[4];
	  	 	ADNS3080MotionDataj.ShutterLower=rxdataj[5];
	  	 	ADNS3080MotionDataj.MaximumPixel=rxdataj[6];
	  	 	xj=rxdataj[1];
	  	 				/*xj -= 1;
	  	 		  	 	  	 xj = ~xj;
	  	 		  	 	  	  xj=(-1)*xj;
	  	 		  	 	  	 xj-=256;*/
	  	 	yj=rxdataj[2];
	  	 		/*yj -= 1;
	  	 	  	 yj = ~yj;
	  	 	  	  yj=(-1)*yj;
	  	 	  	 yj-=256;*/

	  	 	dXj+=(((xj*0.00635)*10)/0.3);
	  	 	dYj+=(((yj*0.00635)*10)/0.3);

	  	 	/*if(y&0x80)
	  	 	  {

	  	 	  //Az y kétkomplementális átalakítása
	  	 	  y -= 1;
	  	 	  y = ~y;
	  	 	  y=(-1)*y;
	  	 	  y-=256;

	  	 	}
	  	 	SumX=SumX+x;             //Felhalmozza az X-et a mobil adatok olvasásához
	  	 	SumY=SumY+y;*/
	  	 	uart_txdataj[0]=dXj;
	  	 	uart_txdataj[1]=dYj;
	  	 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,1); //LD3
	  	 	//HAL_UART_Transmit(&huart2,uart_txdata,2,10);

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
