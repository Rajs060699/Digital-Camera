/********************************
 * Author Rajesh Srirangam
 * Title usart.c
 * Description : Used for USART Transmit and receive data
 * Reference links:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/usart.c
 */


/***********HEADERS***********/
#include "usart.h"	 
#include <stdio.h>
/****************************/

/**********GLOBAL HEADERS************/
unsigned char USART1_ReceiveData;
unsigned char NewCMD = 0;
extern UART_HandleTypeDef huart1;
/***************************************/



/**********************************
 * Function void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT)
 * Param USARTx,USART_IT
 * Return NULL
 * Description Used for clearing pending data from interrupt
 */

void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->SR = (uint16_t)~itmask;                      //Used for clearing Interrupt Flag
}

/**********************************
 * Function void USART_ReceiveData(USART_TypeDef* USARTx)
 * Param USARTx,USART_IT
 * Return NULL
 * Description Used for storing data of UART
 */


uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* Receive Data */
  return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);             //Data stored in DR register
}

/**********************************
 * Function UART1_DataTransmission(uint32_t, uint8_t *)
 * Param len-length of fifo
 * Return data
 * Description Used for receiving USART data
 */
void UART1_DataTransmission(uint32_t len, uint8_t *p)
{
	uint8_t *a=p;
	uint32_t data_count =0;
    while(data_count<=len)
	{
		while(((huart1.Instance)->SR & (UART_FLAG_TXE))==RESET);            //Used for checking Transmit Flag is received or not
		HAL_UART_Transmit(&huart1,a,16,100);                                //Data Transmitted Via UART
		a++;
		data_count++;
	}
}



/**********************************
 * Function void USART1_IRQHandler(void)
 * Param NULL
 * Return NULL
 * Description Used for Handling USART Interrupt
 */

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, UART_IT_RXNE) != RESET)   //Check whether Interrupt occured or not
	{
		USART_ClearITPendingBit(USART1, UART_IT_RXNE);    //Check whether Pending Interrupt Bit is cleared or not
		USART1_ReceiveData = USART_ReceiveData(USART1);   //Data received from SPI DMA passed to UART
		NewCMD = 1;                                       //Flag set if data is received
	}
}


