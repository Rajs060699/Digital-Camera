/****************************
 * Author Rajesh Srirangam
 * Title usart.h
 * Description Used for USART Initialisation
 * Reference links:https://github.com/ArduCAM/STM32/blob/master/STM32F103/HardWare/usart.h
 */



/***********HEADERS***********/
#include "main.h"
/***************************/

/********GLOBAL HAEDERS*************/
extern unsigned char USART1_ReceiveData;
extern unsigned char NewCMD;
/************************************/

/*********FUNCTION PROTOTYPES**********/
void USART1_UART_Init(uint32_t bound);
void UART1_DataTransmission(uint32_t len, uint8_t *p);
ITStatus USART_GetITStatus(USART_TypeDef*, uint16_t);
/****************************************/
