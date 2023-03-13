/******************************
 * Author Rajesh Srirangam
 * Title main.c
 * Description :Used for intialising the functions
 * Reference links:https://github.com/ArduCAM/STM32/blob/master/STM32F103/User/main.c
 */


/************HEADERS***********/
#include <I2C.h>
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "ArduCAM.h"
#include "spi.h"
#include "usart.h"
/***************************/

/*********HANDLE TYPE DEF***********/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
/***********************************/

/***********Function Prototypes***********/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/***************************************/

uint8_t vid, pid;              //vid~video ID and pid~picture ID
uint8_t camera_test = 0;       //camera test modes like capture or video mode
uint8_t start_flag = 0;        //start flag to start capture or video mode
uint8_t stop_flag=0;           //stop flag to stop video or capture mode

/****************************
 * Function : void delay_us()
 * Param uint16_t
 * Return NULL
 * Description Used for creating micro-seciond delay
 */
void delay_us(uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<delay);
}

int main(void)
{

  HAL_Init();                    //HAL Initialisation
  SystemClock_Config();          //System Clock Initialisation
  MX_GPIO_Init();                //GPIO Intialisation
  MX_DMA_Init();                 //DMA Initialisation
  MX_SPI1_Init();                //SPI1 Initialisation
  MX_USART1_UART_Init();         //USART1 Initialisation
  MX_USART2_UART_Init();         //USART2 Initialisation
  MX_TIM1_Init();                //TIM1 Initialisation

  while (1)
		{
			sensor_addr = 0x60;                      //Camera Module Sensor Address Initiated
			wrSensorReg8_8(0xff, 0x01);              //
			rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid); //Used for reading VID value
			rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);  //Used for reading PID value
			if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) //OV2640 Module Not detected
				printf("ACK CMD Can't find OV2640 module!\r\n");
			else
			{
			  sensor_model =  OV2640 ;                         //OV2640 Module Not detected
			  printf("ACK CMD OV2640 detected.\r\n");
			  break;
			}
		}
	  ArduCAM_Init(sensor_model);     //If module detected will be calling this ArduCAM Initialisation function

		while(1)
		{
			if(NewCMD == 1)                  //If data is received from UART
			{
				NewCMD = 0;
				switch(USART1_ReceiveData)
				{
					case 1:
						if(sensor_model== OV2640)         //If model detected is OV2640 will set JPEG Mode
						{
							OV2640_set_JPEG_size();
							printf("ACK CMD switch to OV2640_320x240\r\n");
						}
						break;
					case 0x10:                         // If 0x10 is detected will be used to capture JPEG photo
						camera_test = 1;
						start_flag = 1;
						printf("ACK CMD CAM start single shoot.\r\n");
						break;
					case 0x11:                       // If 0x11 ,set camera to JPEG output mode.
						set_format(JPEG);
						ArduCAM_Init(sensor_model);
						break;
					case 0x20:                     //If 0x20 is received will capture JPEG photo and write data continously to FIFO buffer
						camera_test = 2;
						start_flag = 2;
						printf("ACK CMD CAM start video streaming.\r\n");
						break;
					case 0x21:                      //If 0x21 is received will stop_flag data transfer to FIFO buffer
						stop_flag = 1;
						printf("ACK CMD CAM stop_flag video streaming.\r\n");
						break;
					case 0x30:                                //if0x30 is received video Mode will be on and also JPEG will be captured
						camera_test = 3;
						start_flag = 3;
						printf("ACK CMD CAM start single shoot.\r\n");
						break;
					    default:
						break;
				}
			}
			if(camera_test == 1)
			{
				if(start_flag == 1)
				{
					start_flag = 0;
					SingleCapTransfer();               //Image View will be done here
				}
				if(receive_OK)
				{
					receive_OK= 0;
					SendbyUSART1();                //Send data to UART
				}
			}
			else if(camera_test == 2)
			{
				if(start_flag == 2)
				{
					if(send_OK)
					{
						if(stop_flag)
						{
							printf("ACK CMD CAM stop_flag video streaming.\r\n");
							stop_flag = 0;
							camera_test = 0;
							start_flag = 0;
						}
						    send_OK=false;
						    SingleCapTransfer();         //Image View will be done here
					}
					if(receive_OK)
					{
						    receive_OK= 0;
						    SendbyUSART1();             //Send data to UART
					}
				}
			}
			else if(camera_test == 3)        //For capturing video mode
			{
				if(start_flag == 3)
				{
					 start_flag = 0;
					 camera_test = 0;
					 Start_JPEG_Capture();     //Video capture done here
				}
			}
		}
  }

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;               //SPI is set to bidirecetional mode
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;                   //SPI data size of 8 bits
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;                 //SPI serial clock is set to LOW state
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;                     //For rising edge data is captured
  hspi1.Init.NSS = SPI_NSS_SOFT;                             //Slave Select bit is enabled though software
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;   //SPI Serial Clock is enabled for Transmit or Receive
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;                    //SPI First Bit is set to MSB
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;                    //SPI Transmission Interrupt is diabled
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;    //CRC Calicaulation is disabled
  hspi1.Init.CRCPolynomial = 10;                             //CRC_Polynomial is set to 10
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};                  //Clocksource config is set to 0
  TIM_MasterConfigTypeDef sMasterConfig = {0};                      //Master Config is set to 0
  htim1.Instance = TIM1;                                            //TIM1 is enabled
  htim1.Init.Prescaler = 0;                                         //Prescalar=0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;                      //Up counter is enabled
  htim1.Init.Period = 65535;                                        //Period is set to 65535
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;                //Timer Clock Division is set to 0
  htim1.Init.RepetitionCounter = 0;                                 //Repetition counter is disabled
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;    //Autoreload mode is disabled
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;             //Clock source is disabled
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;                            //Timer Trigger Mode is reset
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;                   //Master Slave Mode is disabled
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
/******************************
 * Function void MX_USART1_UART_Init()
 * Param NULL
 * Return NULL
 * Description Used for USART1 Initialisation
 */

static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;                    //Baudrate Initiated
  huart1.Init.WordLength = UART_WORDLENGTH_8B;      //Word Length 8 bits
  huart1.Init.StopBits = UART_STOPBITS_1;           //StopBits 1 is set
  huart1.Init.Parity = UART_PARITY_NONE;            //Parity Bits set to none
  huart1.Init.Mode = UART_MODE_TX_RX;               //Transmission and receive mode is enabled
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;      //Hardware control none
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;  //Oversampling is set to 16
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/******************************
 * Function void MX_USART2_UART_Init()
 * Param NULL
 * Return NULL
 * Description Used for USART1 Initialisation
 */

static void MX_USART2_UART_Init(void)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;                    //Baudrate Initiated
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;      //Word Length 8 bits
	  huart2.Init.StopBits = UART_STOPBITS_1;           //StopBits 1 is set
	  huart2.Init.Parity = UART_PARITY_NONE;            //Parity Bits set to none
	  huart2.Init.Mode = UART_MODE_TX_RX;               //Transmission and receive mode is enabled
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;      //Hardware control none
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;  //Oversampling is set to 16
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}
/*****************************
 * Function MX_DMA_Init()
 * Param NULL
 * Return NULL
 * Description Used for DMA Intialisation
 */

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}
/***************************
 * Function void MX_GPIO_Init()
 * Return NULL
 * Param NULL
 * Description Used for GPIO Initialisation
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2C_SDA_Pin|I2C_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;             //Output Mode is enabled
  GPIO_InitStruct.Pull = GPIO_NOPULL;                     //No Push Pull mode is enabled
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;           //Low Frequency Mode is enabled
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_SDA_Pin I2C_SCL_Pin */
  GPIO_InitStruct.Pin = I2C_SDA_Pin|I2C_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;                //Output Mode is enabled
  GPIO_InitStruct.Pull = GPIO_NOPULL;                        //No Push Pull Mode enabled
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;               //Low Frequency Mode is enabled
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


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
