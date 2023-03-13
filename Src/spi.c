/******************************
 * Author Rajesh Srirangam
 * Title spi.c
 * Description :Used for SPI Initialisation
 * Reference links:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/spi.c
 */

/*******HEADERS************/
#include "spi.h"
#include "ArduCAM.h"
#include "main.h"
#include "stdint.h"
/****************************/

/**********HANDLE TYPES***********/
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern UART_HandleTypeDef huart1;
/*********************************/

/**********GLOBAL HEADERS*************/
uint8_t	*picbuf = 0;
bool receive_OK = false;    
bool send_OK = true;
uint32_t sendlen = 0;
uint32_t haveRev = 0;
uint32_t noRev = 0;
uint8_t	Buf1[BUFFER_MAX_SIZE]={0}, Buf2[BUFFER_MAX_SIZE]={0};
/****************************************/


/*********************************
 * Function uint8_t SPI1_ReadWriteByte(uint8_t TxData)
 * Param  TxData
 * Return TxData
 * Description Used for Transmitting and receiving SPI data
 */
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)  //Until SPI Ready State data is not Tranmsitted
	HAL_SPI_Transmit(&hspi1, &TxData, 1, 1000);    //SPI Tranmission is done
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)//Until SPI Ready State data is not Received
	HAL_SPI_Receive(&hspi1, &TxData, 1, 1000);     //SPI Receive is done
	return TxData;
}

/*********************************
 * Function DMA1_RX(uint8_t *p , uint32_t len)
 * Param  data and length
 * Return NULL
 * Description Used for SPI data through DMA
 */
void DMA1_RX(uint8_t *p , uint32_t len)
{
	 CS_LOW();                          //Chip Select made low
	 set_fifo_burst();                  //Fifo burst enabled
	 DMA1_Channel2->CMAR = (uint32_t)p; //SPI DMA data register
	 DMA1_Channel2->CNDTR = len;      //SPI DMA Datasize register
	 HAL_DMA_Init(&hdma_spi1_rx);      //DMA initialisation is done
}

/*********************************
 * Function void SendbyUSART1(void)
 * Param  NULL
 * Return NULL
 * Description Used for Sending by DMA data to UART
 */

void SendbyUSART1(void)
{	
	uint8_t	*sdbuf;
	haveRev += sendlen;
    if(haveRev < length)
	{	
		if(picbuf == Buf1)                    //Check if picture buffer size is equal to buffer length or not
		{		
			sdbuf = Buf1;	  picbuf = Buf2;	
		}
		else
		{
			sdbuf = Buf2;	  picbuf = Buf1;
		}
		UART1_DataTransmission(sendlen,sdbuf);          //Send data to UART
		noRev	= length - haveRev;		
		sendlen	= (noRev>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : noRev;	
		DMA1_RX(picbuf, sendlen);	                 //Send data to SPI DMA
	}
	else
	{
		UART1_DataTransmission(sendlen, picbuf);           //Send data to UART
		send_OK = 1;                                        //Send_Ok flag is enabled
	}			 	 					 	 	
}
/*********************************
 * Functionvoid SingleCapTransfer(void)
 * Param  NULL
 * Return NULL
 * Description Used for image capturing
 */

void SingleCapTransfer(void)
{
	flush_fifo();                                                    //fifo reset to zero
	clear_fifo_flag();                                              //clear captured fifo flag
	start_capture();                                                //Initialise capture command
	while(!get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)){;}               //Used for setting trigger bit for Arducam
	length= read_fifo_length();
	sendlen = (length>=BUFFER_MAX_SIZE) ? BUFFER_MAX_SIZE : length;  //Used for checking buffer size with length
	picbuf = Buf1;
	haveRev = 0;
	DMA1_RX(picbuf, sendlen);                                       //Send data through SPI DMA
}
/*********************************
 * Function void Start_JPEG_Capture(void)
 * Param  NULL
 * Return NULL
 * Description Used for JPEG capture
 */

void Start_JPEG_Capture(void)
{
	uint8_t *p1;
	uint8_t a=0xcc;
	uint8_t *p2;
	uint8_t b=0xbb;
	uint8_t *p3;
	uint8_t c=0xaa;
	uint8_t *p4;
	uint8_t d=0xff;
	flush_fifo();                                                 //Used for reset the fifo pointer to zero
	clear_fifo_flag();                                            //used for clearing capture flag
	start_capture();                                              //Used for initialisation of capture commands
	while(!get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)){;}            //Used for setting trigger bit for Arducam
	length= read_fifo_length();                                 //Read current capture length
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)==RESET)    //Checks till Tranmsit flag will go to reset condition
		p3=&c;
	HAL_UART_Transmit(&huart1, p3,16,100);                          //Data Transmission done via UART
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)==RESET)    //Checks till Tranmsit flag will go to reset condition
		p4=&d;
	HAL_UART_Transmit(&huart1, p4,16,100);                         //Data Transmission done via UART
	CS_LOW();                                                      //Chip Select Made Low
	set_fifo_burst();                                              //fifo burst mode is enabled
	uint8_t VH, VL;
	int i = 0, j = 0;
	for (i = 0; i < 240; i++)                                         //For 320*240 Pixel Range
	{
		for (j = 0; j < 320; j++)
		{
			VH = SPI1_ReadWriteByte(0x00);
			VL = SPI1_ReadWriteByte(0x00);
			while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)==RESET);  //Checks till Tranmsit flag will go to reset condition
			uint8_t *vidbuf= &VL;
			HAL_UART_Transmit(&huart1, vidbuf,16,100);                  //Data Transmission done via UART
			delay_us(15);                                               //15 microseconds delay
			while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)==RESET);  //Checks till Tranmsit flag will go to reset condition
			uint8_t *vidbuf1= &VH;
			HAL_UART_Transmit(&huart1,vidbuf1,16,100);                  //Data Transmission done via UART
			delay_us(15);                                                 //15 microseconds delay
		}
	}
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)==RESET);     //Checks till Tranmsit flag will go to reset condition
    p2=&b;
	HAL_UART_Transmit(&huart1, p2,16,100);                         //Data Transmission done via UART
	delay_us(12);                                                  //12 microseconds delay
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)==RESET);      //Checks till Tranmsit flag will go to reset condition
	p1= &a;
	HAL_UART_Transmit(&huart1, p1,16,100);                           //Data Transmission done via UART
	CS_HIGH();                                                       //Chip Select High
}
/*************************
 *Function void DMA1_Channel2_IRQHandler(void)
 *Return NULL
 *Param NULL
 *Description Used for clear interrupt flag of SPI DMA
 */
void DMA1_Channel2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA_FLAG_TC2))
	{
		__HAL_DMA_CLEAR_FLAG(DMA1,DMA_FLAG_TC2);  //Used for clearing flag
		 __HAL_DMA_DISABLE(&hdma_spi1_rx) ;       //SPI DMA Disable
		 HAL_SPI_DMAStop(&hspi1);                 //SPI DMA data transmission is stopped
		CS_HIGH();                                //Chip Select is High
		receive_OK=1;                             //Enable Receive Ok flag
	}
}

